./bin\RelWithDebInfo/ypathtrace.exe tests/01_surface/surface.json -o out/lowres/01_surface_720_256.jpg -t volpath -s 256 -r 720
./bin\RelWithDebInfo/ypathtrace.exe tests/02_rollingteapot/rollingteapot.json -o out/lowres/02_rollingteapot_720_256.jpg -t volpath -s 256 -r 720
./bin\RelWithDebInfo/ypathtrace.exe tests/03_volume/volume.json -o out/lowres/03_volume_720_256.jpg -t volpath -s 256 -r 720
./bin\RelWithDebInfo/ypathtrace.exe tests/04_head1/head1.json -o out/lowres/04_head1_720_256.jpg -t volpath -s 256 -r 720
./bin\RelWithDebInfo/ypathtrace.exe tests/05_head1ss/head1ss.json -o out/lowres/05_head1ss_720_256.jpg -t volpath -s 256 -r 720

./bin\RelWithDebInfo/ypathtrace_adp.exe tests/06_adaptive/06_adaptive.json -o out/adaptive_rendering/quality_0.jpg -t volpath -s 256 -r 720 -q 0
./bin\RelWithDebInfo/ypathtrace_adp.exe tests/06_adaptive/06_adaptive.json -o out/adaptive_rendering/quality_1.jpg -t volpath -s 256 -r 720 -q 1
./bin\RelWithDebInfo/ypathtrace_adp.exe tests/06_adaptive/06_adaptive.json -o out/adaptive_rendering/quality_2.jpg -t volpath -s 256 -r 720 -q 2
./bin\RelWithDebInfo/ypathtrace_adp.exe tests/06_adaptive/06_adaptive.json -o out/adaptive_rendering/quality_3.jpg -t volpath -s 256 -r 720 -q 3
./bin\RelWithDebInfo/ypathtrace_adp.exe tests/06_adaptive/06_adaptive.json -o out/adaptive_rendering/quality_4.jpg -t volpath -s 256 -r 720 -q 4
./bin\RelWithDebInfo/ypathtrace_adp.exe tests/06_adaptive/06_adaptive.json -o out/adaptive_rendering/quality_5.jpg -t volpath -s 256 -r 720 -q 5
./bin\RelWithDebInfo/ypathtrace_adp.exe tests/06_adaptive/06_adaptive.json -o out/adaptive_rendering/quality_6.jpg -t volpath -s 256 -r 720 -q 6