#include "../include/NaoExercise.h"
#include <unistd.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "myo_converter");

    NaoExercise nao_exercise;
    ros::spin();

    return 0;
}

