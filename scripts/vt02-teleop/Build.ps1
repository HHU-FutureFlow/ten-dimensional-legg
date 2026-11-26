$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$outputDirectory = Join-Path $repoRoot "build/vt02-teleop"
$outputFile = Join-Path $outputDirectory "WheellegTeleop.exe"
$compiler = Join-Path $env:WINDIR "Microsoft.NET/Framework64/v4.0.30319/csc.exe"
if (-not (Test-Path -LiteralPath $compiler -PathType Leaf)) {
    $compiler = Join-Path $env:WINDIR "Microsoft.NET/Framework/v4.0.30319/csc.exe"
}
if (-not (Test-Path -LiteralPath $compiler -PathType Leaf)) {
    throw "The Windows .NET Framework 4.x C# compiler was not found."
}

New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
& $compiler /nologo /target:winexe /optimize+ /codepage:65001 `
    /reference:System.Windows.Forms.dll /reference:System.Drawing.dll `
    "/out:$outputFile" (Join-Path $PSScriptRoot "WheellegTeleop.cs")
if ($LASTEXITCODE -ne 0) {
    throw "WheellegTeleop build failed with exit code $LASTEXITCODE."
}
Write-Host $outputFile
