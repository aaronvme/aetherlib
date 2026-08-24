<#
Builds and runs the PROS-free host test suite with MSVC.

    .\test\host\run.ps1
    .\test\host\run.ps1 -Filter geometry
    .\test\host\run.ps1 -Rebuild
    .\test\host\run.ps1 -Tests mcl -OutDir $env:TEMP\mcl   # private build dir
#>
[CmdletBinding()]
param(
    [string]$Filter = "",
    [string]$Tests = "",
    [string]$OutDir = "",
    [switch]$Rebuild
)

$ErrorActionPreference = "Stop"

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = Resolve-Path (Join-Path $here "..\..")
$out = if ($OutDir) { $OutDir } else { Join-Path $here "build" }

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) { throw "vswhere.exe not found; MSVC build tools are required" }
$vsPath = & $vswhere -products * -latest -format value -property installationPath
if (-not $vsPath) { throw "no Visual Studio / Build Tools installation found" }
$vcvars = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"
if (-not (Test-Path $vcvars)) { throw "vcvars64.bat not found under $vsPath" }

if ($Rebuild -and (Test-Path $out)) { Remove-Item -Recurse -Force $out }
New-Item -ItemType Directory -Force -Path $out | Out-Null

# chad_test.cpp holds main() and the registry, so it is always compiled in.
$sources = @(
    Get-ChildItem -Path $here -Filter "*.cpp" |
        Where-Object { -not $Tests -or $_.Name -eq "chad_test.cpp" -or $_.Name -like "*$Tests*" } |
        Sort-Object Name | ForEach-Object { $_.FullName }
)
if ($sources.Count -le 1) { throw "no test sources in $here matching '$Tests'" }

# Library sources that do not touch PROS can be linked in and tested for real.
# The check is transitive: a file with no pros/ include of its own still fails to
# compile on the host if any header it reaches pulls one in.
$incRoot = Join-Path $root "include"
$prosFreeCache = @{}

function Test-ProsFree {
    param([string]$Path)

    $key = $Path.ToLowerInvariant()
    if ($prosFreeCache.ContainsKey($key)) { return $prosFreeCache[$key] }
    $prosFreeCache[$key] = $true   # cycle guard

    foreach ($line in (Get-Content -LiteralPath $Path)) {
        if ($line -notmatch '^\s*#\s*include\s*[<"]([^">]+)[">]') { continue }
        $inc = $Matches[1]
        if ($inc -like 'pros/*' -or $inc -like 'liblvgl/*' -or
            $inc -eq 'api.h' -or $inc -eq 'main.h' -or $inc -like 'gamespecific/*') {
            $prosFreeCache[$key] = $false
            return $false
        }
        if ($inc -like 'chad/*') {
            $resolved = Join-Path $incRoot $inc
            if ((Test-Path $resolved) -and -not (Test-ProsFree -Path $resolved)) {
                $prosFreeCache[$key] = $false
                return $false
            }
        }
    }
    $prosFreeCache[$key] = $true
    return $true
}

$libSources = @(
    Get-ChildItem -Path (Join-Path $root "src\chad") -Filter "*.cpp" -Recurse |
        Where-Object { Test-ProsFree -Path $_.FullName } |
        Sort-Object FullName | ForEach-Object { $_.FullName }
)
$sources += $libSources
Write-Host "host-testable library sources: $($libSources.Count)"

$exe = Join-Path $out "chad_host_tests.exe"
$flags = @(
    "/nologo", "/std:c++latest", "/EHsc", "/W4", "/WX", "/permissive-",
    "/fp:precise", "/Zc:preprocessor", "/O2", "/DCHAD_HOST_TEST=1",
    "/I`"$root\include`"", "/I`"$here`""
)

$quoted = ($sources | ForEach-Object { "`"$_`"" }) -join " "
$cmd = "cl $($flags -join ' ') /Fo:`"$out\\`" /Fe:`"$exe`" $quoted"
$build = cmd /c "call `"$vcvars`" >nul 2>&1 && $cmd" 2>&1
$buildOk = $LASTEXITCODE -eq 0

$build | Where-Object { $_ -notmatch '^\s*$' -and $_ -notmatch '\.cpp$' } | ForEach-Object { Write-Host $_ }

if (-not $buildOk) {
    Write-Host "HOST BUILD FAILED" -ForegroundColor Red
    exit 1
}

& $exe $Filter
exit $LASTEXITCODE
