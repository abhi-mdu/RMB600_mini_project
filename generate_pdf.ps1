# PDF Generation Script
# Converts the comprehensive report to PDF with proper formatting

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "  COMPREHENSIVE REPORT PDF GENERATOR" -ForegroundColor Cyan
Write-Host "========================================`n" -ForegroundColor Cyan

# Check if pandoc is installed
$pandocInstalled = Get-Command pandoc -ErrorAction SilentlyContinue

if (-not $pandocInstalled) {
    Write-Host "[!] Pandoc not found. Installing..." -ForegroundColor Yellow
    Write-Host "    Please install Pandoc from: https://pandoc.org/installing.html" -ForegroundColor Yellow
    Write-Host "    Or use chocolatey: choco install pandoc" -ForegroundColor Yellow
    Write-Host "`n    After installing, run this script again.`n" -ForegroundColor Yellow
    exit 1
}

Write-Host "[OK] Pandoc found: $($pandocInstalled.Version)`n" -ForegroundColor Green

# Generate PDF
Write-Host "Generating PDF from FINAL_COMPREHENSIVE_REPORT.md..." -ForegroundColor Cyan

$pdfCommand = @"
pandoc FINAL_COMPREHENSIVE_REPORT.md -o FINAL_REPORT.pdf ``
       --pdf-engine=xelatex ``
       --resource-path=.:figures ``
       --toc --toc-depth=3 ``
       --number-sections ``
       --highlight-style=tango ``
       -V geometry:margin=1in ``
       -V fontsize=11pt ``
       -V documentclass=article ``
       -V colorlinks=true ``
       -V linkcolor=blue ``
       -V urlcolor=blue
"@

try {
    pandoc FINAL_COMPREHENSIVE_REPORT.md -o FINAL_REPORT.pdf `
           --pdf-engine=xelatex `
           --resource-path=.:figures `
           --toc --toc-depth=3 `
           --number-sections `
           --highlight-style=tango `
           -V geometry:margin=1in `
           -V fontsize=11pt `
           -V documentclass=article `
           -V colorlinks=true `
           -V linkcolor=blue `
           -V urlcolor=blue
    
    if (Test-Path "FINAL_REPORT.pdf") {
        $pdfInfo = Get-Item "FINAL_REPORT.pdf"
        Write-Host "`n[SUCCESS] PDF generated!" -ForegroundColor Green
        Write-Host "  File: FINAL_REPORT.pdf" -ForegroundColor White
        Write-Host "  Size: $([math]::Round($pdfInfo.Length/1MB, 2)) MB" -ForegroundColor White
        Write-Host "  Created: $($pdfInfo.CreationTime)`n" -ForegroundColor White
        
        # Open the PDF
        Write-Host "Opening PDF..." -ForegroundColor Cyan
        Start-Process "FINAL_REPORT.pdf"
    }
} catch {
    Write-Host "`n[ERROR] PDF generation failed:" -ForegroundColor Red
    Write-Host "  $($_.Exception.Message)`n" -ForegroundColor Red
    
    Write-Host "Alternative: Generate DOCX instead" -ForegroundColor Yellow
    Write-Host "Command: pandoc FINAL_COMPREHENSIVE_REPORT.md -o FINAL_REPORT.docx --toc`n" -ForegroundColor Yellow
}

# Also generate PowerPoint
Write-Host "`nGenerating PowerPoint presentation..." -ForegroundColor Cyan

try {
    pandoc FINAL_COMPREHENSIVE_REPORT.md -o FINAL_PRESENTATION.pptx `
           --resource-path=.:figures `
           -t pptx `
           --slide-level=2
    
    if (Test-Path "FINAL_PRESENTATION.pptx") {
        $pptxInfo = Get-Item "FINAL_PRESENTATION.pptx"
        Write-Host "`n[SUCCESS] PowerPoint generated!" -ForegroundColor Green
        Write-Host "  File: FINAL_PRESENTATION.pptx" -ForegroundColor White
        Write-Host "  Size: $([math]::Round($pptxInfo.Length/1KB, 2)) KB" -ForegroundColor White
        Write-Host "  Slides: ~40 slides (review and adjust)`n" -ForegroundColor White
    }
} catch {
    Write-Host "`n[WARNING] PowerPoint generation failed" -ForegroundColor Yellow
    Write-Host "  You can create slides manually from the report`n" -ForegroundColor Yellow
}

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  GENERATION COMPLETE" -ForegroundColor Cyan
Write-Host "========================================`n" -ForegroundColor Cyan

Write-Host "Generated Files:" -ForegroundColor White
Get-ChildItem -Filter "FINAL_*" | ForEach-Object {
    Write-Host "  [OK] $($_.Name) ($([math]::Round($_.Length/1KB, 2)) KB)" -ForegroundColor Green
}

Write-Host "`nNext Steps:" -ForegroundColor Cyan
Write-Host "  1. Review FINAL_REPORT.pdf" -ForegroundColor White
Write-Host "  2. Adjust FINAL_PRESENTATION.pptx if needed" -ForegroundColor White
Write-Host "  3. Submit to course portal" -ForegroundColor White
Write-Host "`n"
