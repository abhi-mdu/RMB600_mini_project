# Media Files Verification Script
# Checks if all required figures and videos have been generated

Write-Host ""
Write-Host "================================" -ForegroundColor Cyan
Write-Host " MEDIA FILES VERIFICATION" -ForegroundColor Cyan
Write-Host "================================" -ForegroundColor Cyan
Write-Host ""

# Initialize counters
$figuresFound = 0
$figuresMissing = 0
$videosFound = 0
$videosMissing = 0
$totalSize = 0

# Check Figures
Write-Host "Checking Figures..." -ForegroundColor Yellow
Write-Host ""

$requiredFigures = @(
    "fig1_error_message.png",
    "fig2_quaternion_visualization.png",
    "fig3_test_results.png",
    "fig4_movel_path.png",
    "fig5_slerp_comparison.png",
    "fig6_movej_path.png",
    "fig7_comparison.png",
    "fig8_frame_hierarchy.png",
    "fig9_pentagon_path.png",
    "fig10_performance_chart.png"
)

$missingFigures = @()

foreach ($fig in $requiredFigures) {
    $path = "figures\$fig"
    if (Test-Path $path) {
        $size = (Get-Item $path).Length / 1KB
        $totalSize += $size
        $sizeStr = [math]::Round($size, 1)
        Write-Host "  [OK] $fig ($sizeStr KB)" -ForegroundColor Green
        $figuresFound++
    } else {
        Write-Host "  [MISSING] $fig" -ForegroundColor Red
        $missingFigures += $fig
        $figuresMissing++
    }
}

Write-Host ""
Write-Host "Checking Videos..." -ForegroundColor Yellow
Write-Host ""

$requiredVideos = @(
    "video1_full_simulation.mp4",
    "video2_comparison.mp4",
    "video3_tests.mp4"
)

$missingVideos = @()

foreach ($vid in $requiredVideos) {
    $path = "videos\$vid"
    if (Test-Path $path) {
        $size = (Get-Item $path).Length / 1MB
        $totalSize += ($size * 1024)
        $sizeStr = [math]::Round($size, 1)
        Write-Host "  [OK] $vid ($sizeStr MB)" -ForegroundColor Green
        $videosFound++
    } else {
        Write-Host "  [MISSING] $vid" -ForegroundColor Red
        $missingVideos += $vid
        $videosMissing++
    }
}

# Summary
Write-Host ""
Write-Host "================================" -ForegroundColor Cyan
Write-Host " SUMMARY" -ForegroundColor Cyan
Write-Host "================================" -ForegroundColor Cyan
Write-Host ""

Write-Host "Figures: $figuresFound/$($requiredFigures.Count) found" -ForegroundColor $(if ($figuresMissing -eq 0) { "Green" } else { "Yellow" })
Write-Host "Videos:  $videosFound/$($requiredVideos.Count) found" -ForegroundColor $(if ($videosMissing -eq 0) { "Green" } else { "Yellow" })
Write-Host "Total Size: $([math]::Round($totalSize / 1024, 1)) MB" -ForegroundColor Cyan
Write-Host ""

# Status
if ($figuresMissing -eq 0 -and $videosMissing -eq 0) {
    Write-Host "[SUCCESS] ALL MEDIA FILES PRESENT!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Next Steps:" -ForegroundColor Yellow
    Write-Host "  1. Open PowerPoint and create presentation" -ForegroundColor White
    Write-Host "  2. Insert figures using MEDIA_INSERTION_GUIDE.md" -ForegroundColor White
    Write-Host "  3. Embed videos on demo slides" -ForegroundColor White
    Write-Host "  4. Convert article to PDF" -ForegroundColor White
    Write-Host "  5. Practice presentation (aim for 45-60 minutes)" -ForegroundColor White
} else {
    Write-Host "[WARNING] INCOMPLETE - Missing Files" -ForegroundColor Yellow
    Write-Host ""
    
    if ($missingFigures.Count -gt 0) {
        Write-Host "Missing Figures:" -ForegroundColor Red
        foreach ($fig in $missingFigures) {
            Write-Host "  - $fig" -ForegroundColor White
        }
        Write-Host ""
    }
    
    if ($missingVideos.Count -gt 0) {
        Write-Host "Missing Videos:" -ForegroundColor Red
        foreach ($vid in $missingVideos) {
            Write-Host "  - $vid" -ForegroundColor White
        }
        Write-Host ""
    }
    
    Write-Host "Next Steps:" -ForegroundColor Yellow
    Write-Host "  1. Open MATLAB" -ForegroundColor White
    Write-Host "  2. Run: generate_all_media.m (for figures)" -ForegroundColor White
    Write-Host "  3. Run: robot_simulation.m (capture remaining figures + video)" -ForegroundColor White
    Write-Host "  4. Run: compare_movej_movel.m (capture comparison)" -ForegroundColor White
    Write-Host "  5. Record videos during execution" -ForegroundColor White
    Write-Host ""
    Write-Host "See MANUAL_EXECUTION_INSTRUCTIONS.md for details" -ForegroundColor Cyan
}

Write-Host ""
Write-Host "================================" -ForegroundColor Cyan
Write-Host ""
