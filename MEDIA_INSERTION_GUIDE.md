# Quick Reference: Inserting Figures and Videos into Presentation and Article

## For PowerPoint Presentation (PRESENTATION.md → PPT)

### Converting Markdown to PowerPoint

**Option 1: Using Pandoc**
```bash
pandoc PRESENTATION.md -o PRESENTATION.pptx
```

**Option 2: Manual Creation**
1. Open PowerPoint
2. Create slides following PRESENTATION.md structure
3. Insert figures/videos as indicated below

### Figure Insertion Locations

| Slide # | Placeholder Text | Insert File | Notes |
|---------|------------------|-------------|-------|
| 3 | [INSERT FIGURE: error_message.png] | figures/fig1_error_message.png | Before implementation |
| 7 | [INSERT FIGURE: quaternion_viz.png] | figures/fig2_quaternion_visualization.png | Math visualization |
| 8 | [INSERT FIGURE: test_results.png] | figures/fig3_test_results.png | All tests passing |
| 10 | [INSERT FIGURE: movel_trajectory.png] | figures/fig4_movel_path.png | Straight red path |
| 10 | [INSERT FIGURE: slerp_comparison.png] | figures/fig5_slerp_comparison.png | SLERP validation |
| 12 | [INSERT FIGURE: movej_trajectory.png] | figures/fig6_movej_path.png | Curved blue path |
| 13 | [INSERT FIGURE: trajectory_comparison.png] | figures/fig7_comparison.png | Side-by-side |
| 14 | [INSERT FIGURE: frame_hierarchy.png] | figures/fig8_frame_hierarchy.png | Frame tree |
| 15 | [INSERT FIGURE: pentagon_path.png] | figures/fig9_pentagon_path.png | Complete motion |
| 17 | [INSERT FIGURE: performance_chart.png] | figures/fig10_performance_chart.png | Bar chart |
| 26 | [INSERT VIDEO: full_demo.mp4] | videos/video1_full_simulation.mp4 | 2:20 duration |
| 26 | [INSERT VIDEO: comparison.mp4] | videos/video2_comparison.mp4 | 1:05 duration |

### PowerPoint Insertion Instructions

**For Images**:
1. Navigate to slide
2. Click: Insert → Pictures → This Device
3. Browse to figures/ folder
4. Select appropriate file
5. Resize to fit slide (maintain aspect ratio)
6. Position centrally
7. Optional: Add border/shadow for polish

**For Videos**:
1. Navigate to slide
2. Click: Insert → Video → This Device
3. Browse to videos/ folder
4. Select video file
5. Set to "Start: Automatically" or "On Click"
6. Resize to appropriate size
7. Test playback in Slide Show mode

**Quality Settings**:
- Image compression: Use "Best quality for printing" (Options → Compress Pictures)
- Video quality: Keep original (don't compress)
- Slide size: Widescreen 16:9 (Design → Slide Size)

---

## For Article (COMPREHENSIVE_ARTICLE.md)

### If Converting to PDF (Recommended)

**Using Pandoc with LaTeX**:
```bash
# Install pandoc and LaTeX first (MiKTeX on Windows)
pandoc COMPREHENSIVE_ARTICLE.md -o COMPREHENSIVE_ARTICLE.pdf --pdf-engine=xelatex

# With figures embedded:
pandoc COMPREHENSIVE_ARTICLE.md -o COMPREHENSIVE_ARTICLE.pdf \
  --pdf-engine=xelatex \
  --include-in-header=header.tex \
  -V geometry:margin=1in
```

**Create header.tex for better formatting**:
```latex
\usepackage{graphicx}
\usepackage{float}
\usepackage{hyperref}
```

### Manual Figure Insertion in Markdown

Replace placeholder comments with actual markdown image syntax:

**Example Replacements**:

```markdown
<!-- BEFORE -->
*Figure 1: Error message showing undefined functions*

<!-- AFTER -->
![Figure 1: Error message showing undefined functions](figures/fig1_error_message.png)
```

**All Replacement Locations**:

| Section | Original Text | Replacement |
|---------|---------------|-------------|
| §2.2 | *Figure showing quaternion conversion* | `![Quaternion Conversion](figures/fig2_quaternion_visualization.png)` |
| §5.1.3 | *See test results* | `![Test Results](figures/fig3_test_results.png)` |
| §5.2.3 | *MoveL produces straight paths* | `![MoveL Straight Path](figures/fig4_movel_path.png)` |
| §5.2.3 | *SLERP provides constant velocity* | `![SLERP Comparison](figures/fig5_slerp_comparison.png)` |
| §5.3.4 | *MoveJ produces curved paths* | `![MoveJ Curved Path](figures/fig6_movej_path.png)` |
| §6.3 | *Trajectory comparison* | `![Trajectory Comparison](figures/fig7_comparison.png)` |
| §5.4.1 | *Frame hierarchy diagram* | `![Frame Hierarchy](figures/fig8_frame_hierarchy.png)` |
| §5.4.3 | *Pentagon drawing sequence* | `![Pentagon Path](figures/fig9_pentagon_path.png)` |
| §6.1 | *Performance comparison* | `![Performance Chart](figures/fig10_performance_chart.png)` |

### Adding Video Links in Markdown

Since PDFs don't embed videos, add links:

```markdown
**Video Demonstration**: [Watch full simulation](videos/video1_full_simulation.mp4)

**Comparison Video**: [MoveJ vs MoveL comparison](videos/video2_comparison.mp4)
```

Or if publishing online (HTML):
```markdown
<video width="640" height="480" controls>
  <source src="videos/video1_full_simulation.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
```

---

## Batch Insertion Script (PowerShell)

Save as `insert_media.ps1`:

```powershell
# PowerShell script to prepare presentation with media

Write-Host "=== Media Insertion Helper ===" -ForegroundColor Cyan

# Check if figures exist
Write-Host "`nChecking figures..." -ForegroundColor Yellow
$figures = @(
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

$missing = @()
foreach ($fig in $figures) {
    $path = "figures\$fig"
    if (Test-Path $path) {
        Write-Host "  ✓ $fig" -ForegroundColor Green
    } else {
        Write-Host "  ✗ MISSING: $fig" -ForegroundColor Red
        $missing += $fig
    }
}

# Check if videos exist
Write-Host "`nChecking videos..." -ForegroundColor Yellow
$videos = @(
    "video1_full_simulation.mp4",
    "video2_comparison.mp4",
    "video3_tests.mp4"
)

foreach ($vid in $videos) {
    $path = "videos\$vid"
    if (Test-Path $path) {
        $size = (Get-Item $path).Length / 1MB
        Write-Host "  ✓ $vid ($("{0:N1}" -f $size) MB)" -ForegroundColor Green
    } else {
        Write-Host "  ✗ MISSING: $vid" -ForegroundColor Red
        $missing += $vid
    }
}

# Summary
Write-Host "`n=== Summary ===" -ForegroundColor Cyan
if ($missing.Count -eq 0) {
    Write-Host "All media files present! ✓" -ForegroundColor Green
    Write-Host "`nNext steps:" -ForegroundColor Yellow
    Write-Host "  1. Open PowerPoint"
    Write-Host "  2. Create slides from PRESENTATION.md"
    Write-Host "  3. Insert figures at marked locations"
    Write-Host "  4. Insert videos on demo slides"
    Write-Host "  5. Test all animations and transitions"
} else {
    Write-Host "Missing $($missing.Count) files:" -ForegroundColor Red
    foreach ($m in $missing) {
        Write-Host "  - $m"
    }
    Write-Host "`nGenerate missing files using SIMULATION_RUN_GUIDE.md" -ForegroundColor Yellow
}

Write-Host ""
```

Run with:
```powershell
.\insert_media.ps1
```

---

## Quality Checklist

### Before Presentation

- [ ] All 10 figures inserted
- [ ] All 2+ videos embedded
- [ ] Videos play correctly in Slide Show mode
- [ ] Images are high resolution (not pixelated)
- [ ] Animations/transitions work
- [ ] Font sizes readable from distance
- [ ] Color scheme consistent
- [ ] Speaker notes added
- [ ] Timing practiced (45-60 min)
- [ ] Backup copy saved

### Before Article Submission

- [ ] All figures embedded or linked
- [ ] Figure captions numbered correctly
- [ ] References to figures use correct numbers
- [ ] Video links work (if online version)
- [ ] PDF renders correctly
- [ ] Page breaks at appropriate places
- [ ] Table of contents generated
- [ ] Bibliography formatted
- [ ] Headers/footers correct
- [ ] Proofread for typos

---

## File Size Management

### If Presentation File Too Large

**Reduce Image Size**:
```matlab
% In MATLAB, export at lower resolution
exportgraphics(fig, 'output.png', 'Resolution', 150); % Instead of 300
```

**Compress Videos**:
```bash
# Using ffmpeg (install first)
ffmpeg -i input.mp4 -crf 28 -preset slow output_compressed.mp4
```

**PowerPoint Compression**:
1. File → Info
2. Compress Media → Standard Quality (OK for projection)

### Recommended Sizes

| Media Type | Max Size | Recommended |
|------------|----------|-------------|
| Individual Figure | 5 MB | 1-2 MB |
| Individual Video | 50 MB | 20-30 MB |
| Complete PPT | 200 MB | 100-150 MB |
| Article PDF | 20 MB | 5-10 MB |

---

## Presentation Tips

### Slide Transition Recommendations

- **Title/Section slides**: Fade (0.5s)
- **Content slides**: None or Subtle (0.3s)
- **Demo slides with videos**: None (let video be focus)
- **Comparison slides**: Wipe or Push (0.5s)

### Animation Recommendations

- **Bullet points**: Appear (click), 0.3s
- **Figures**: Fade in, 0.5s
- **Important stats**: Grow/Shrink for emphasis
- **Videos**: Auto-play on slide entry

### Speaking Points for Each Figure

**Fig 1 (Error)**: "This error motivated our entire implementation..."

**Fig 2 (Quaternion)**: "We use quaternion mathematics because..."

**Fig 3 (Tests)**: "Comprehensive testing validates every function..."

**Fig 4 (MoveL)**: "Linear motion maintains straight paths..."

**Fig 5 (SLERP)**: "SLERP provides constant angular velocity..."

**Fig 6 (MoveJ)**: "Joint-space motion is faster but curved..."

**Fig 7 (Comparison)**: "Side-by-side shows the key differences..."

**Fig 8 (Frames)**: "Our hierarchical frame system mirrors RAPID..."

**Fig 9 (Pentagon)**: "The complete sequence draws a perfect pentagon..."

**Fig 10 (Performance)**: "MoveJ achieves 15x speedup through optimization..."

---

## Troubleshooting

### PowerPoint Issues

**Problem**: Video won't play
- **Solution**: Convert to WMV or embed using "Insert → Video → This Device"

**Problem**: Images blurry
- **Solution**: Use original high-res files, don't compress

**Problem**: File too large
- **Solution**: Use "Compress Media" feature in PowerPoint

### PDF Issues

**Problem**: Figures not showing
- **Solution**: Check markdown image paths are relative (e.g., `figures/fig1.png`)

**Problem**: Layout broken
- **Solution**: Add page breaks with `\newpage` in markdown

**Problem**: Links not clickable
- **Solution**: Use proper markdown link syntax `[text](url)`

---

## Final Checklist

### Presentation Ready When:
- [x] PRESENTATION.md converted to PPT
- [x] All 10 figures inserted
- [x] 2 videos embedded and tested
- [x] Slide numbers added
- [x] Transitions configured
- [x] Practiced timing
- [x] Backup saved (USB + cloud)

### Article Ready When:
- [x] COMPREHENSIVE_ARTICLE.md complete
- [x] All figures embedded/linked
- [x] Figure references correct
- [x] PDF generated and reviewed
- [x] Table of contents accurate
- [x] References formatted
- [x] Proofread thoroughly
- [x] Final PDF < 10 MB

---

**Quick Start**: 
1. Run `SIMULATION_RUN_GUIDE.md` instructions
2. Generate all figures and videos
3. Run `insert_media.ps1` to verify
4. Follow this guide to insert into presentation/article
5. Review and practice!

**Estimated Time**: 30-45 minutes for insertion after media generation

**Document Version**: 1.0  
**Last Updated**: January 15, 2026
