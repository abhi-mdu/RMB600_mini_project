# 🌐 MATLAB Online Execution Guide
## Minimal Setup for Generating All Media Files

**Goal**: Run simulation on MATLAB Online, download media, create presentation locally  
**Time Required**: ~30 minutes  
**Storage**: 0 GB local (until download)

---

## 📦 Step 1: Files to Upload to MATLAB Online

### Essential Files (11 files total)

Upload these to MATLAB Online:

```
📁 Files to Upload:
├── robot_simulation.m          (Main simulation - REQUIRED)
├── test_functions.m            (Test suite - REQUIRED)
├── compare_movej_movel.m       (Comparison tool - REQUIRED)
├── generate_media_online.m     (NEW - Created below)
└── robot/                      (Robot model - REQUIRED)
    ├── test.urdf               (Robot definition)
    └── IRB1600/                (7 STL mesh files)
        ├── base.stl
        ├── link1.stl
        ├── link2.stl
        ├── link3.stl
        ├── link4.stl
        ├── link5.stl
        └── link6.stl
```

### How to Upload to MATLAB Online

1. Go to: https://matlab.mathworks.com
2. Sign in with MathWorks account (create if needed - it's free)
3. Click **Upload** button (top left)
4. Select all 4 MATLAB files (.m files)
5. Create folder: Right-click → New Folder → name it `robot`
6. Enter `robot` folder, upload `test.urdf`
7. In `robot` folder, create subfolder `IRB1600`
8. Enter `IRB1600` folder, upload all 7 .stl files

---

## 🚀 Step 2: Automated Execution Script

I'll create a streamlined script for MATLAB Online that generates everything automatically.

### Quick Execution (Recommended)

Once files are uploaded, run this single command in MATLAB Online:

```matlab
generate_media_online
```

This will:
- ✅ Generate all 6 automated figures
- ✅ Run tests and capture results
- ✅ Run simulation (you manually export 3 figures)
- ✅ Run comparison (you manually export 1 figure)
- ✅ Create all output files

### Manual Execution (Alternative)

If you prefer step-by-step control:

```matlab
% Step 1: Generate automated figures (5 min)
run_figure_generation

% Step 2: Run tests
test_functions

% Step 3: Run simulation (capture fig4, fig6, fig9 manually)
robot_simulation

% Step 4: Run comparison (capture fig7 manually)
compare_movej_movel
```

---

## 📥 Step 3: Download Generated Media

After execution completes:

### Files to Download (13 files)

**Figures (10 PNG files)**:
1. `fig1_error_message.png`
2. `fig2_quaternion_visualization.png`
3. `fig3_test_results.png`
4. `fig4_movel_path.png` (manual capture from simulation)
5. `fig5_slerp_comparison.png`
6. `fig6_movej_path.png` (manual capture from simulation)
7. `fig7_comparison.png` (manual capture from comparison)
8. `fig8_frame_hierarchy.png`
9. `fig9_pentagon_path.png` (manual capture from simulation)
10. `fig10_performance_chart.png`

**Videos (3 MP4 files)** - Optional, use screen recording:
- `video1_full_simulation.mp4` (record robot_simulation.m execution)
- `video2_comparison.mp4` (record compare_movej_movel.m execution)
- `video3_tests.mp4` (record test_functions output)

### How to Download from MATLAB Online

**For individual files**:
1. Right-click on file → Download

**For all figures at once**:
```matlab
% In MATLAB Online, create a zip file
zip('all_figures.zip', 'fig*.png')
% Then download all_figures.zip
```

**For folders**:
1. Select all files in folder
2. Right-click → Download

---

## 💻 Step 4: Local Presentation Creation

After downloading media files to your local machine:

1. **Place files in correct folders**:
   ```
   d:\Masters\Robotics\mini_project\
   ├── figures\
   │   └── (10 PNG files)
   └── videos\
       └── (3 MP4 files - if recorded)
   ```

2. **Verify files**:
   ```powershell
   .\verify_media.ps1
   ```

3. **Create presentation**:
   - Follow MEDIA_INSERTION_GUIDE.md
   - Convert PRESENTATION.md to PowerPoint
   - Insert downloaded figures

4. **Create article**:
   ```bash
   pandoc COMPREHENSIVE_ARTICLE.md -o COMPREHENSIVE_ARTICLE.pdf --pdf-engine=xelatex
   ```

---

## ⏱️ Timeline

| Task | Location | Time |
|------|----------|------|
| Upload files | MATLAB Online | 5 min |
| Run generation script | MATLAB Online | 10 min |
| Manual captures | MATLAB Online | 10 min |
| Download media | MATLAB Online → Local | 5 min |
| Verify files | Local | 2 min |
| Create presentation | Local | 45 min |
| **Total** | - | **~75 min** |

---

## 🎯 Execution Checklist

### On MATLAB Online:
- [ ] Upload 4 MATLAB scripts
- [ ] Upload robot/ folder (URDF + 7 STL files)
- [ ] Run: `generate_media_online`
- [ ] Manually capture 4 figures during execution
- [ ] Optional: Record 3 videos with screen capture
- [ ] Download all generated PNG files
- [ ] Download videos (if recorded)

### On Local Machine:
- [ ] Place figures in figures/ folder
- [ ] Place videos in videos/ folder
- [ ] Run: `.\verify_media.ps1`
- [ ] Create PowerPoint presentation
- [ ] Convert article to PDF
- [ ] Review and practice

---

## 🆘 Troubleshooting MATLAB Online

### Issue: Files don't upload
**Solution**: Upload in smaller batches, or upload one-by-one

### Issue: Out of storage space
**Solution**: MATLAB Online has 5GB storage. Delete temporary files:
```matlab
delete('*.asv')  % Delete autosave files
```

### Issue: Simulation too slow
**Solution**: MATLAB Online uses shared resources. Try off-peak hours.

### Issue: Can't export figures
**Solution**: Use right-click → Export or:
```matlab
exportgraphics(gcf, 'filename.png', 'Resolution', 300);
```

### Issue: Need to download everything at once
**Solution**: 
```matlab
% Zip everything
zip('all_media.zip', '*.png')
% Download the zip file
```

---

## 💡 Pro Tips for MATLAB Online

1. **Use Tab Completion**: Type first few letters, press Tab
2. **Keep Browser Tab Active**: Prevents session timeout
3. **Save Often**: MATLAB Online auto-saves, but be safe
4. **Use Split Screen**: View code and output simultaneously
5. **Check Storage**: Use `!ls -lh` to see file sizes

---

## 📊 What You'll Have After This

**On MATLAB Online** (temporary):
- All generated figure files
- Robot simulation results
- Test outputs

**On Local Machine** (permanent):
- 10 high-quality figures (300 DPI PNG)
- 3 demonstration videos (optional)
- Complete presentation (PowerPoint)
- Technical article (PDF)
- All documentation

---

## 🔄 Alternative: Quick Figure Generation Only

If you only need figures (no videos), use this ultra-fast method:

```matlab
% In MATLAB Online
quick_generate_all_figures  % 5 minutes total
zip('figures.zip', 'fig*.png')
% Download figures.zip
```

Then on local machine, create presentation without videos.

---

**Next Step**: I'll create the `generate_media_online.m` script for you now.
