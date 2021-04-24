# IKProject

you need to install RBDL by using dev branch : https://github.com/rbdl/rbdl

Checkout the dev branch:

git checkout dev
• Build and install RBDL with the luamodel-Plugin:

mkdir build
cd build
cmake .. -DRBDL_BUILD_ADDON_LUAMODEL=ON
make
sudo make install



meshup info:
red axes -> positive X axes
green axes -> positive Y axes
blue axes -> positive Z axes
