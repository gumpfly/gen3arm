g++  JacoArmCtrl.cpp JacoArm.cpp -lpthread -std=c++11 -I ./ -ldl -L../kortex_api/lib/release -lKortexApi -L/home/lyg/dragonfly-master/lib -lDragonfly -o JacoArm
# g++ getposition.cpp JacoArmCtrl.cpp -lpthread -std=c++11 -I ./ -ldl -L/home/bcilab/dragonfly-master/lib -lDragonfly -o getposition
