#mogrify -format png -crop 8x8+1+1 +repage -path tag36h11_cropped tag36h11/*.png
# if not cropping:
mkdir -p tag16h5_mod
cp tag16h5/*.png tag16h5_mod
mogrify -fuzz 1% -fill "rgb(96, 210, 255)" -opaque white tag16h5_mod/*.png
