BGRRAW-1756410547786929_120: Thursday, 28 August 2025 19:49:07.786
BGRRAW-1756410547886929_120: Thursday, 28 August 2025 19:49:07.886
BGRRAW-1756410547986929_120: Thursday, 28 August 2025 19:49:07.986
BGRRAW-1756410549086929_120: Thursday, 28 August 2025 19:49:09.086

Images 100ms

# Useful Commands

Convert simulator *RAW* images to video:

```bash
ffmpeg -video_size 1920x1080 -f image2 -pattern_type glob -framerate 10 -pixel_format bgr24 -i 'data/CAMERA/BGRRAW-175641054*.raw' out.mp4
```

Play video using `ffplay`:

```bash
ffplay out.mp4
```