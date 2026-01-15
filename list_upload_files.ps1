# List Files for MATLAB Online Upload
# Run this to get the exact files you need to upload

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host " FILES TO UPLOAD TO MATLAB ONLINE" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Phase 1: MATLAB Scripts
Write-Host "PHASE 1: MATLAB Scripts (4 files)" -ForegroundColor Yellow
Write-Host ""

$matlab_files = @(
    "robot_simulation.m",
    "test_functions.m",
    "compare_movej_movel.m",
    "generate_media_online.m"
)

foreach ($file in $matlab_files) {
    if (Test-Path $file) {
        $size = (Get-Item $file).Length / 1KB
        Write-Host "  [OK] $file ($([math]::Round($size, 1)) KB)" -ForegroundColor Green
    } else {
        Write-Host "  [MISSING] $file" -ForegroundColor Red
    }
}

# Phase 2: Robot URDF
Write-Host ""
Write-Host "PHASE 2: Robot URDF (1 file)" -ForegroundColor Yellow
Write-Host ""

$urdf_file = "robot\test.urdf"
if (Test-Path $urdf_file) {
    $size = (Get-Item $urdf_file).Length / 1KB
    Write-Host "  [OK] $urdf_file ($([math]::Round($size, 1)) KB)" -ForegroundColor Green
} else {
    Write-Host "  [MISSING] $urdf_file" -ForegroundColor Red
}

# Phase 3: STL Files
Write-Host ""
Write-Host "PHASE 3: STL Mesh Files (7 files)" -ForegroundColor Yellow
Write-Host ""

$stl_files = @(
    "robot\IRB1600\base.stl",
    "robot\IRB1600\link1.stl",
    "robot\IRB1600\link2.stl",
    "robot\IRB1600\link3.stl",
    "robot\IRB1600\link4.stl",
    "robot\IRB1600\link5.stl",
    "robot\IRB1600\link6.stl"
)

$total_stl_size = 0
foreach ($file in $stl_files) {
    if (Test-Path $file) {
        $size = (Get-Item $file).Length / 1KB
        $total_stl_size += $size
        Write-Host "  [OK] $file ($([math]::Round($size, 1)) KB)" -ForegroundColor Green
    } else {
        Write-Host "  [MISSING] $file" -ForegroundColor Red
    }
}

# Summary
Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host " SUMMARY" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

$all_files = $matlab_files + @($urdf_file) + $stl_files
$found_count = 0
$missing_count = 0

foreach ($file in $all_files) {
    if (Test-Path $file) {
        $found_count++
    } else {
        $missing_count++
    }
}

Write-Host "Files Found: $found_count / $($all_files.Count)" -ForegroundColor $(if ($missing_count -eq 0) { "Green" } else { "Yellow" })

if ($missing_count -eq 0) {
    # Calculate total size
    $total_size = 0
    foreach ($file in $all_files) {
        $total_size += (Get-Item $file).Length
    }
    
    Write-Host "Total Size: $([math]::Round($total_size / 1MB, 2)) MB" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "[SUCCESS] All files ready for upload!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Next Steps:" -ForegroundColor Yellow
    Write-Host "  1. Go to https://matlab.mathworks.com" -ForegroundColor White
    Write-Host "  2. Sign in (create free account if needed)" -ForegroundColor White
    Write-Host "  3. Upload 4 MATLAB scripts (.m files)" -ForegroundColor White
    Write-Host "  4. Create folder 'robot', upload test.urdf" -ForegroundColor White
    Write-Host "  5. Create folder 'robot/IRB1600', upload 7 .stl files" -ForegroundColor White
    Write-Host "  6. Run: generate_media_online" -ForegroundColor White
    Write-Host ""
    Write-Host "See UPLOAD_CHECKLIST.md for detailed instructions" -ForegroundColor Cyan
} else {
    Write-Host ""
    Write-Host "[WARNING] $missing_count file(s) missing!" -ForegroundColor Red
    Write-Host ""
    Write-Host "Missing Files:" -ForegroundColor Red
    foreach ($file in $all_files) {
        if (-not (Test-Path $file)) {
            Write-Host "  - $file" -ForegroundColor White
        }
    }
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""
