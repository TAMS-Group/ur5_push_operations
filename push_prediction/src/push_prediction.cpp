#include <ros/ros.h>
#include <ros/package.h>
#include <push_prediction/neural_network.h>

int main(int argc, char** argv)
{
	NeuralNetwork network;
	network.load(ros::package::getPath("push_prediction") + "scripts/keras_model.yaml");
	return 0;
}
