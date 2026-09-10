@echo off

@rem with paletten
@rem ffmpeg -ss 0 -t 3 -i Nuvoton_MCU.mp4 -vf "fps=4,scale=426:240,crop=320:240:(iw-320)/2:0,palettegen=max_colors=16" -y palette.png
@rem ffmpeg -ss 0 -t 3 -i Nuvoton_MCU.mp4 -i palette.png -lavfi "fps=4,scale=426:240,crop=320:240:(iw-320)/2:0[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=5" -y nuvoton.gif

ffmpeg -i Nuvoton_MCU.mp4 -vf "fps=10,scale=426:240,crop=320:240:(iw-320)/2:0" nuvoton.gif

python gif2c.py nuvoton.gif