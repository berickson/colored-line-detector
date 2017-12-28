set -e
./rebuild-platform.sh
cd pbuild
make jvpkg
cd ..
echo usbsd >> /dev/ttyACM0
echo "giving time for usb to mount"
sleep 5
mv *.jvpkg /media/$USER/JEVOIS/packages
cp config/videomappings.cfg /media/brian/JEVOIS/config/
cp config/params.cfg /media/brian/JEVOIS/config/
cp config/initscript.cfg /media/brian/JEVOIS/config/
sync
echo sync >> /dev/ttyACM0
echo "about to restart"
sleep 1
echo "restarting"
echo restart >> /dev/ttyACM0
