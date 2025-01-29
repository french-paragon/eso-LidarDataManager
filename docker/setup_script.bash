apt update; DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt install -y build-essential cmake git libeigen3-dev libproj-dev libtclap-dev cimg-dev

mkdir buildcpp; cd buildcpp;

git clone https://github.com/french-paragon/MultidimArrays.git; mkdir build-multidim; cd build-multidim; cmake -DCMAKE_BUILD_TYPE=Release ../MultidimArrays; make; make install; cd ..
git clone https://github.com/french-paragon/LibStevi.git; mkdir build-stevi; cd build-stevi; cmake -DCMAKE_BUILD_TYPE=Release -DbuildTests=OFF -DbuildExamples=OFF -DbuildGui=OFF ../LibStevi; make; make install; cd ..

cd ..
rm -rf buildcpp #cleanup after build;

apt remove -y git #we do not need git
