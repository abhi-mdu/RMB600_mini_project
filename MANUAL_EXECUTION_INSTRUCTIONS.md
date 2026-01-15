# 📋 MANUAL EXECUTION INSTRUCTIONS
## How to Generate All Figures and Videos

Since MATLAB needs to be run manually, follow these steps:

---

## ⚡ Quick Start (5 Steps)

### Step 1: Open MATLAB
1. Launch MATLAB from Start Menu
2. Navigate to project folder:
   ```matlab
   cd 'd:\Masters\Robotics\mini_project'
   ```

### Step 2: Run Automated Figure Generation
```matlab
% Run the automated script
generate_all_media
```

This will create 6 figures automatically:
- ✅ fig1_error_message.png
- ✅ fig2_quaternion_visualization.png
- ✅ fig3_test_results.png
- ✅ fig5_slerp_comparison.png
- ✅ fig8_frame_hierarchy.png
- ✅ fig10_performance_chart.png

### Step 3: Run Tests
```matlab
% Run test suite
test_functions
```

### Step 4: Generate Remaining Figures with Simulation

**Important**: Before running, start screen recording for videos!

```matlab
% Run full simulation
robot_simulation
```

**Manual Captures During Simulation**:

1. **After MoveJ motion** (blue curve):
   - Rotate the 3D view to show the curved path clearly
   - File → Export Setup → Export → Save as `figures/fig6_movej_path.png`

2. **After Pentagon complete**:
   - Rotate view to show complete pentagon
   - Export as `figures/fig9_pentagon_path.png`

3. **MoveL trajectory**:
   - After any MoveL motion (red line)
   - Export as `figures/fig4_movel_path.png`

### Step 5: Run Comparison Tool
```matlab
% Generate comparison figure
compare_movej_movel
```

Then export the figure as `figures/fig7_comparison.png`

---

## 📹 Recording Videos

### Video 1: Full Simulation (2-3 minutes)

**Setup**:
1. Open screen recording software (OBS Studio, Camtasia, or Windows Game Bar: Win+G)
2. Position MATLAB figure window to fill most of screen
3. Start recording

**Steps**:
```matlab
% Run simulation
robot_simulation
```

**What to capture**:
- 0:00-0:10 - Robot model loading
- 0:10-0:20 - MoveJ motion (blue, fast, curved)
- 0:20-1:30 - Pentagon drawing (red MoveL paths)
- 1:30-2:00 - Rotate 3D view to show trajectories from different angles
- 2:00-2:20 - Zoom in on details

**Save as**: `videos/video1_full_simulation.mp4`

### Video 2: Comparison (1 minute)

**Setup**:
1. Start screen recording
2. Run comparison

**Steps**:
```matlab
compare_movej_movel
```

**What to capture**:
- Side-by-side motion comparison
- Timing differences
- Path shape differences (curved vs straight)

**Save as**: `videos/video2_comparison.mp4`

### Video 3: Tests Running (30 seconds)

**Setup**:
1. Clear command window: `clc`
2. Start recording command window
3. Run tests

**Steps**:
```matlab
test_functions
```

**What to capture**:
- Test output scrolling
- Green checkmarks appearing
- Final "ALL TESTS PASSED" message

**Save as**: `videos/video3_tests.mp4`

---

## 📊 Verification Checklist

After completing all steps, verify:

### Figures (10 files)
- [ ] figures/fig1_error_message.png
- [ ] figures/fig2_quaternion_visualization.png
- [ ] figures/fig3_test_results.png
- [ ] figures/fig4_movel_path.png
- [ ] figures/fig5_slerp_comparison.png
- [ ] figures/fig6_movej_path.png
- [ ] figures/fig7_comparison.png
- [ ] figures/fig8_frame_hierarchy.png
- [ ] figures/fig9_pentagon_path.png
- [ ] figures/fig10_performance_chart.png

### Videos (3 files)
- [ ] videos/video1_full_simulation.mp4
- [ ] videos/video2_comparison.mp4
- [ ] videos/video3_tests.mp4

### Run Verification Script
```powershell
# In PowerShell terminal
.\verify_media.ps1
```

---

## 🚨 Troubleshooting

### If robot_simulation.m gives errors:

**Error: Cannot find URDF**
```matlab
% Check file exists
if exist('robot/test.urdf', 'file')
    disp('✓ URDF found')
else
    disp('✗ URDF missing - check path')
    disp('Expected: robot/test.urdf')
end
```

**Error: Robotics Toolbox not found**
```matlab
% Check toolbox
ver('robotics')
```

If not installed, install from MATLAB Add-Ons.

### If figures export fails:

**Alternative export method**:
```matlab
% Instead of exportgraphics, use:
print(gcf, 'figures/filename.png', '-dpng', '-r300')
```

### If videos are too large:

Use video compression tools or reduce resolution in recording software.

---

## ⏱️ Expected Timeline

| Task | Time |
|------|------|
| Run generate_all_media.m | 5 min |
| Run robot_simulation.m + captures | 15 min |
| Run compare_movej_movel.m | 3 min |
| Record video 1 | 5 min |
| Record video 2 | 3 min |
| Record video 3 | 2 min |
| **Total** | **~35 min** |

---

## 📝 After Generation

Once all media is generated:

1. **Verify files**: Run `.\verify_media.ps1` in PowerShell
2. **Review quality**: Open each figure and video to check quality
3. **Create presentation**: Follow MEDIA_INSERTION_GUIDE.md
4. **Create article PDF**: Follow QUICK_START.md

---

## 🎯 Next Steps

After generating all media:

```matlab
% In MATLAB, verify everything is ready
fprintf('Checking generated media...\n');
fprintf('Figures: %d files\n', length(dir('figures/*.png')));
fprintf('Videos: %d files\n', length(dir('videos/*.mp4')));
fprintf('\nReady to create presentation!\n');
fprintf('See MEDIA_INSERTION_GUIDE.md for next steps.\n');
```

---

**Questions?** See:
- SIMULATION_RUN_GUIDE.md - Detailed generation instructions
- MEDIA_INSERTION_GUIDE.md - How to use generated media
- QUICK_START.md - Fast-track overview

**Good luck!** 🚀
