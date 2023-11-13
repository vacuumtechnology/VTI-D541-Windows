#include "CreateModel.h"

using namespace std;

int main (int argc, char *argv[])
{
	string filename = argv[1];
	CreateModel* model = new CreateModel(filename);


	model->Configure();


	delete(model);
	return (0);
}