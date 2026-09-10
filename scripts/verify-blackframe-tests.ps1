param(
    [Parameter(Mandatory = $true)][string]$DependencyRoot,
    [string]$MavenCommand = 'mvn',
    [string]$JavaHome,
    [string]$SettingsFile
)

# Run Ashtrace's integration tests, then unmodified copies of selected lower-layer tests.
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path $PSScriptRoot -Parent
$taskDependencyRoot = (Resolve-Path -LiteralPath $DependencyRoot).Path
$taskSelection = Get-Content -LiteralPath "$PSScriptRoot/blackframe-contract-tests.json" -Raw | ConvertFrom-Json
$taskInputs = foreach ($taskRelative in $taskSelection) {
    $taskSource = Join-Path $taskDependencyRoot $taskRelative
    @{ Relative = $taskRelative; Source = $taskSource; Sha256 = (Get-FileHash -LiteralPath $taskSource -Algorithm SHA256).Hash }
}

& "$PSScriptRoot/verify-local.ps1" -DependencyRoot $taskDependencyRoot -MavenCommand $MavenCommand `
        -JavaHome $JavaHome -SettingsFile $SettingsFile

# Each run gets a separate directory, so stale copied tests cannot join a later run.
$taskRun = Join-Path $taskRoot ('.verification/blackframe-tests/run-' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $taskRun -Force | Out-Null
foreach ($taskInput in $taskInputs) {
    $taskPackagePath = $taskInput.Relative.Split(@('/src/test/java/'), [StringSplitOptions]::None)[1]
    $taskCopy = Join-Path $taskRun "src/test/java/$taskPackagePath"
    New-Item -ItemType Directory -Path (Split-Path $taskCopy -Parent) -Force | Out-Null
    Copy-Item -LiteralPath $taskInput.Source -Destination $taskCopy
    if ((Get-FileHash -LiteralPath $taskCopy -Algorithm SHA256).Hash -ne $taskInput.Sha256) {
        throw "Test source changed during copying: $($taskInput.Relative)"
    }
}
$taskInputs | ConvertTo-Json | Set-Content -LiteralPath "$taskRun/imports.json" -Encoding UTF8

# Copy Ashtrace's pinned dependencies and test build configuration into a standalone harness.
# Only the test phase runs here: this harness is neither packaged nor published.
[xml]$taskProject = Get-Content -LiteralPath "$taskRoot/pom.xml" -Raw
$taskVersion = $taskProject.project.version
$taskProperties = $taskProject.project.properties.OuterXml.Replace(' xmlns="http://maven.apache.org/POM/4.0.0"', '')
$taskDependencies = $taskProject.project.dependencies.OuterXml.Replace(' xmlns="http://maven.apache.org/POM/4.0.0"', '')
$taskPlugins = ($taskProject.project.build.plugins.plugin | Where-Object {
    $_.artifactId -in @('maven-resources-plugin', 'maven-compiler-plugin', 'maven-surefire-plugin')
} | ForEach-Object { $_.OuterXml.Replace(' xmlns="http://maven.apache.org/POM/4.0.0"', '') }) -join "`n"
$taskPom = @"
<project xmlns="http://maven.apache.org/POM/4.0.0">
    <modelVersion>4.0.0</modelVersion>
    <groupId>dev.nasaka.blackframe</groupId>
    <artifactId>ashtrace-blackframe-contract-tests</artifactId>
    <version>$taskVersion</version>
    <name>Blackframe dependency contract tests for Ashtrace</name>
    $taskProperties
    $taskDependencies
    <build><plugins>$taskPlugins</plugins></build>
</project>
"@
Set-Content -LiteralPath "$taskRun/pom.xml" -Value $taskPom -Encoding UTF8
if ($SettingsFile) {
    $taskSettings = (Resolve-Path -LiteralPath $SettingsFile).Path
} else {
    $taskSettings = Join-Path $taskRoot '.verification/verification-settings.xml'
}
$taskPreviousJavaHome = $env:JAVA_HOME
try {
    if ($JavaHome) { $env:JAVA_HOME = (Resolve-Path -LiteralPath $JavaHome).Path }
    & $MavenCommand -B -ntp -nsu -s $taskSettings "-Dmaven.repo.local=$taskRoot/.verification/repository" `
            -f "$taskRun/pom.xml" test
    if ($LASTEXITCODE -ne 0) { throw "Blackframe contract tests failed; reports: $taskRun/target/surefire-reports" }
} finally {
    $env:JAVA_HOME = $taskPreviousJavaHome
    foreach ($taskInput in $taskInputs) {
        if ((Get-FileHash -LiteralPath $taskInput.Source -Algorithm SHA256).Hash -ne $taskInput.Sha256) {
            throw "Original test source changed during verification: $($taskInput.Relative)"
        }
    }
}
Write-Output "Unmodified Blackframe tests passed; reports and source hashes: $taskRun"
