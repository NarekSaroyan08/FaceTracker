cat << 'EOF' > README.md
# FaceTracker SDK for RPi4

1. Run setup:
   chmod +x setup_project.sh
   ./setup_project.sh

2. Compile:
   g++ -o FaceTracker FaceTracker.cpp `pkg-config --cflags --libs opencv4` -lpigpio -lpthread -lrt

3. Run:
   sudo ./FaceTracker
EOF# FaceTracker
# FaceTracker
# FaceTracker
# FaceTracker
