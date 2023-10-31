#include "CreateModel.h"

using namespace std;

int main (int argc, char *argv[])
{
	string filename = argv[1];
	CreateModel* model = new CreateModel(filename);

	thread sceneViewer(&CreateModel::Visualizer, model);

	//model->Configure();

	sceneViewer.join();

	delete(model);
	return (0);
}