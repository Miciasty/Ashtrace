param(
    [Parameter(Mandatory = $true)][string]$DependencyRoot,
    [string]$MavenCommand = 'mvn',
    [string]$JavaHome,
    [string]$SettingsFile
)

# Read already-built dependencies; install and verify only inside Ashtrace.
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path $PSScriptRoot -Parent
$taskDependencyRoot = (Resolve-Path -LiteralPath $DependencyRoot).Path
$taskDependencies = Get-Content -LiteralPath "$PSScriptRoot/development-dependencies.json" -Raw | ConvertFrom-Json
$taskFiles = @()
foreach ($taskDependency in $taskDependencies) {
    $taskBase = Join-Path $taskDependencyRoot $taskDependency.directory
    $taskJar = Join-Path $taskBase "target/$($taskDependency.artifactId)-$($taskDependency.version).jar"
    $taskPom = Join-Path $taskBase 'pom.xml'
    if ((Get-FileHash -LiteralPath $taskJar -Algorithm SHA256).Hash -ne $taskDependency.jarSha256) {
        throw "JAR identity mismatch: $taskJar"
    }
    if ((Get-FileHash -LiteralPath $taskPom -Algorithm SHA256).Hash -ne $taskDependency.pomSha256) {
        throw "POM identity mismatch: $taskPom"
    }
    $taskFiles += @{ Jar = $taskJar; Pom = $taskPom }
}

$taskEvidence = Join-Path $taskRoot '.verification'
New-Item -ItemType Directory -Path $taskEvidence -Force | Out-Null
if ($SettingsFile) {
    $taskSettings = (Resolve-Path -LiteralPath $SettingsFile).Path
} else {
    $taskSettings = Join-Path $taskEvidence 'verification-settings.xml'
    Set-Content -LiteralPath $taskSettings -Encoding UTF8 -Value '<settings xmlns="http://maven.apache.org/SETTINGS/1.2.0" />'
}
$taskArguments = @('-B', '-ntp', '-nsu', '-s', $taskSettings, "-Dmaven.repo.local=$taskEvidence/repository")
$taskPreviousJavaHome = $env:JAVA_HOME
Push-Location -LiteralPath $taskRoot
try {
    if ($JavaHome) { $env:JAVA_HOME = (Resolve-Path -LiteralPath $JavaHome).Path }
    & $MavenCommand -version
    if ($LASTEXITCODE -ne 0) { throw 'Cannot run Maven with the selected JDK' }
    foreach ($taskFile in $taskFiles) {
        & $MavenCommand @taskArguments org.apache.maven.plugins:maven-install-plugin:3.1.3:install-file `
                "-Dfile=$($taskFile.Jar)" "-DpomFile=$($taskFile.Pom)"
        if ($LASTEXITCODE -ne 0) { throw "Dependency installation failed: $($taskFile.Jar)" }
    }
    & $MavenCommand @taskArguments clean verify dependency:tree
    if ($LASTEXITCODE -ne 0) { throw 'Ashtrace verification failed' }
} finally {
    $env:JAVA_HOME = $taskPreviousJavaHome
    Pop-Location
}
