set -e
./rebuild-platform.sh
cd pbuild
make jvpkg
cd ..
mv BrianErickson_samplemodule.jvpkg /media/sf_Desktop
