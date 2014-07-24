

#include "LASPointReader.h"

#include <fstream>
#include <iostream>


using std::ifstream;
using std::cout;
using std::endl;

LASPointReader::LASPointReader(string file){
	this->file = file;
	readOpener = new LASreadOpener();
	readOpener->set_file_name(file.c_str());

	reader = readOpener->open();

	Vector3 min = Vector3(reader->get_min_x(), reader->get_min_y(), reader->get_min_z());
	Vector3 max = Vector3(reader->get_max_x(), reader->get_max_y(), reader->get_max_z());
	aabb = AABB(min, max);
}

void LASPointReader::close(){
	reader->close();
}

long LASPointReader::numPoints(){
	return reader->npoints;
}

bool LASPointReader::readNextPoint(){
	return reader->read_point();
}

Point LASPointReader::getPoint(){
	double x = reader->point.get_x();
	double y = reader->point.get_y();
	double z = reader->point.get_z();
	
	char r = (unsigned char)(float(reader->point.get_rgb()[0]));
	char g = (unsigned char)(float(reader->point.get_rgb()[1]));
	char b = (unsigned char)(float(reader->point.get_rgb()[2]));
	unsigned short intensity = reader->point.get_intensity();
	Point p(x,y,z,r,g,b);
	p.intensity = intensity;

	return p;
}

AABB LASPointReader::getAABB(){
	return aabb;
}

Vector3 LASPointReader::getScale(){
	Vector3 scale;
	scale.x = getHeader().x_scale_factor;
	scale.y = getHeader().y_scale_factor;
	scale.z = getHeader().z_scale_factor;

	return scale;
}

LASheader const &LASPointReader::getHeader(){
	return reader->header;
}