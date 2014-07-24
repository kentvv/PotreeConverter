


#include "PotreeConverter.h"
#include "stuff.h"
#include "LASPointReader.h"
#include "PotreeException.h"

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include "lasdefinitions.hpp"
#include "lasreader.hpp"
#include "laswriter.hpp"

#include <chrono>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <math.h>
#include <fstream>

using std::stringstream;
using std::map;
using std::string;
using std::vector;
using std::find;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;
using std::fstream;
using boost::iends_with;

long addAncestorTime = 0;
long addSourceTime = 0;
long saveCloudJSTime = 0;

struct Task{
	string source;
	string target;
	AABB aabb;

	Task(string source, string target, AABB aabb){
		this->source = source;
		this->target = target;
		this->aabb = aabb;
	}
};

PotreeConverter::PotreeConverter(vector<string> fData, string workDir, float minGap, int maxDepth, string format, float range){
	for(int i = 0; i < fData.size(); i++){
		if(!boost::filesystem::exists(fData[i])){
			throw PotreeException("file not found: " + fData[i]);
		}
	}
	

	this->fData = fData;
	this->workDir = workDir;
	this->minGap = minGap;
	this->maxDepth = maxDepth;
	this->format = format;
	this->range = range;
	this->outputFormat = OutputFormat::LAZ;
	buffer = new char[4*10*1000*1000*sizeof(float)];

	boost::filesystem::path dataDir(workDir + "/data");
	boost::filesystem::path tempDir(workDir + "/temp");
	boost::filesystem::create_directories(dataDir);
	boost::filesystem::create_directories(tempDir);

	initReader();
}

void PotreeConverter::initReader(){

	for(int i = 0; i < fData.size(); i++){
		string fname = fData[i];
		LASreadOpener opener;
		opener.set_file_name(fname.c_str());

		if(iends_with(fname, ".las") || iends_with(fname, ".laz")){
			LASreader *reader = opener.open();
			this->reader.insert(std::make_pair(fname, reader));
		}else{
			throw PotreeException("filename did not match a known format");
		}
	}

	currentReader = reader.begin();
}

void PotreeConverter::convert(){
	uint64_t numPoints = 0;
	map<string, LASreader*>::iterator it;
	for(it = reader.begin(); it != reader.end(); it++){
		numPoints += it->second->npoints;
	}
	convert(numPoints);
}

bool PotreeConverter::readNextPoint(){
	bool success = currentReader->second->read_point();

	if(!success){
		currentReader++;
		if(currentReader == reader.end()){
			return false;
		}

		success = currentReader->second->read_point();
	}

	return success;
}

LASpoint &PotreeConverter::getPoint(){
	return currentReader->second->point;
}

string PotreeConverter::getOutputExtension(){
	string outputExtension = "";
	if(outputFormat == OutputFormat::LAS){
		outputExtension = ".las";
	}else if(outputFormat == OutputFormat::LAZ){
		outputExtension = ".laz";
	}

	return outputExtension;
}

void PotreeConverter::convert(uint64_t numPoints){

	{ // calculate aabb
		Vector3 min(std::numeric_limits<double>::max());
		Vector3 max(-std::numeric_limits<double>::max());
		map<string, LASreader*>::iterator it;
		for(it = reader.begin(); it != reader.end(); it++){
			LASreader *reader = it->second;

			Vector3 lmin = Vector3(reader->get_min_x(), reader->get_min_y(), reader->get_min_z());
			Vector3 lmax = Vector3(reader->get_max_x(), reader->get_max_y(), reader->get_max_z());
			min.x = std::min(min.x, lmin.x);
			min.y = std::min(min.y, lmin.y);
			min.z = std::min(min.z, lmin.z);
			max.x = std::max(max.x, lmax.x);
			max.y = std::max(max.y, lmax.y);
			max.z = std::max(max.z, lmax.z);
		}
		aabb = AABB(min, max);

		cout << "AABB: " << endl;
		cout << aabb << endl;
	}
	
	{ // check dimension
		if(minGap == 0.0f){
			double volume = aabb.size.x * aabb.size.y * aabb.size.z;
			minGap = pow(volume, 1.0 / 3.0) * 0.005;
			cout << "automatically calculated spacing: " << minGap << endl;
		}

		double threshold = 10*1000;
		double width = aabb.size.x / minGap;
		double height = aabb.size.y / minGap;
		double depth = aabb.size.z / minGap;
		
		if(width > threshold || height > threshold || depth > threshold){
			cout << endl;
			cout << "WARNING: It seems that either your bounding box is too large or your spacing too small." << endl;
			cout << "Conversion might not work" << endl;
			cout << endl;
		}

	}

	cloudJs.clear();
	cloudJs << "{" << endl;
	cloudJs << "\t" << "\"version\": \"" << POTREE_FORMAT_VERSION << "\"," << endl;
	cloudJs << "\t" << "\"octreeDir\": \"data\"," << endl;
	cloudJs << "\t" << "\"boundingBox\": {" << endl;
	cloudJs << "\t\t" << "\"lx\": " << aabb.min.x << "," << endl;
	cloudJs << "\t\t" << "\"ly\": " << aabb.min.y << "," << endl;
	cloudJs << "\t\t" << "\"lz\": " << aabb.min.z << "," << endl;
	cloudJs << "\t\t" << "\"ux\": " << aabb.max.x << "," << endl;
	cloudJs << "\t\t" << "\"uy\": " << aabb.max.y << "," << endl;
	cloudJs << "\t\t" << "\"uz\": " << aabb.max.z << endl;
	cloudJs << "\t" << "}," << endl;

	if(outputFormat == OutputFormat::BINARY){
		cloudJs << "\t" << "\"pointAttributes\": [" << endl;
		cloudJs << "\t\t" << "\"POSITION_CARTESIAN\"," << endl;
		cloudJs << "\t\t" << "\"COLOR_PACKED\"" << endl;
		cloudJs << "\t" << "]," << endl;
	}else if(outputFormat == OutputFormat::LAS){
		cloudJs << "\t" << "\"pointAttributes\": \"LAS\"," << endl;
	}else if(outputFormat == OutputFormat::LAZ){
		cloudJs << "\t" << "\"pointAttributes\": \"LAZ\"," << endl;
	}
	cloudJs << "\t" << "\"spacing\": " << minGap << "," << endl;
	cloudJs << "\t" << "\"hierarchy\": [" << endl;

	uint64_t numAccepted = 0;
	uint64_t numRejected = 0;
	{ // handle root
		cout << "processing level 0, points: " << numPoints << endl;
		SparseGrid grid(aabb, minGap);
		uint64_t pointsRead = 0;

		//ofstream sdLasOut(workDir + "/temp/d.las", ios::out | ios::binary);
		//ofstream srLasOut(workDir + "/data/r.las", ios::out | ios::binary);

		LASreader *reader = currentReader->second;
		const LASheader &header = reader->header;
		Vector3 scale = Vector3(header.x_scale_factor, header.y_scale_factor, header.z_scale_factor);

		LASheader dHeader;
		dHeader.point_data_format = header.point_data_format;
		dHeader.point_data_record_length = header.point_data_record_length;
		dHeader.x_scale_factor = header.x_scale_factor;
		dHeader.y_scale_factor = header.y_scale_factor;
		dHeader.z_scale_factor = header.z_scale_factor;
		dHeader.min_x = header.min_x;
		dHeader.min_y = header.min_y;
		dHeader.min_z = header.min_z;
		dHeader.max_x = header.max_x;
		dHeader.max_y = header.max_y;
		dHeader.max_z = header.max_z;
		dHeader.x_offset = header.x_offset;
		dHeader.y_offset = header.y_offset;
		dHeader.z_offset = header.z_offset;

		LASheader rHeader;
		rHeader.point_data_format = header.point_data_format;
		rHeader.point_data_record_length = header.point_data_record_length;
		rHeader.x_scale_factor = header.x_scale_factor;
		rHeader.y_scale_factor = header.y_scale_factor;
		rHeader.z_scale_factor = header.z_scale_factor;
		rHeader.min_x = header.min_x;
		rHeader.min_y = header.min_y;
		rHeader.min_z = header.min_z;
		rHeader.max_x = header.max_x;
		rHeader.max_y = header.max_y;
		rHeader.max_z = header.max_z;
		rHeader.x_offset = header.x_offset;
		rHeader.y_offset = header.y_offset;
		rHeader.z_offset = header.z_offset;
		
		LASwriteOpener lwrOpener;
		lwrOpener.set_file_name((workDir + "/data/r" + getOutputExtension()).c_str());
		LASwriter *rWriter = lwrOpener.open(&rHeader);

		LASwriteOpener lwdOpener;
		lwdOpener.set_file_name((workDir + "/temp/d" + getOutputExtension()).c_str());
		LASwriter *dWriter = lwdOpener.open(&dHeader);
		
		
		float minGapAtMaxDepth = minGap / pow(2.0f, maxDepth);
		vector<Point> pAccepted;
		uint64_t i = 0;
		while(readNextPoint()){

			LASpoint &lp = getPoint();

			Point p(lp.get_x(), lp.get_y(), lp.get_z());
			
			bool accepted = grid.add(p);
			int index = nodeIndex(aabb, p);
			if(accepted){
				// write point to ./data/r-file
				try{
					rWriter->write_point(&lp);
					numAccepted++;
				}catch(...){
					cout << "error trying to write to: " << workDir << "/data/r" << getOutputExtension() << endl;
				}
			}else{
				// write point to ./temp/d-file
				try{
					dWriter->write_point(&lp);
					numRejected++;
				}catch(...){
					cout << "error trying to write to: " << workDir << "/data/d" << getOutputExtension() << endl;
				}
			}
			i++;
		}

		cloudJs << "\t\t" << "[\"r\"," << numAccepted << "]," << endl;
		saveCloudJS();

		rWriter->close();
		dWriter->close();
	}

	// close readers
	map<string, LASreader*>::iterator it;
	for(it = reader.begin(); it != reader.end(); it++){
		it->second->close();
	}

	string source = workDir + "/temp/d";
	string target = workDir + "/data/r";
	int depth = 1;
	vector<Task> work;
	uint64_t unprocessedPoints = numRejected;
	uint64_t processedPoints = numAccepted;
	work.push_back(Task(source, target, aabb));

	// process points in breadth first order
	while(!work.empty() && depth <= maxDepth){
		cout << "processing level " << depth << ", points: " << unprocessedPoints << endl;
		vector<Task> nextRound;
		unprocessedPoints = 0;
		processedPoints = 0;

		vector<Task>::iterator it;
		for(it = work.begin(); it != work.end(); it++){
			source = it->source;
			target = it->target;
			AABB aabb = it->aabb;

			ProcessResult result = process(source, target, aabb, depth);

			// prepare the workload of the next level
			unprocessedPoints += result.numRejected;
			processedPoints += result.numAccepted;
			for(int i = 0; i < result.indices.size(); i++){
				int index = result.indices[i];
				stringstream ssSource, ssTarget;
				ssSource << source << result.indices[i];
				ssTarget << target << result.indices[i];
				AABB chAABB = childAABB(aabb, index);
				Task task(ssSource.str(), ssTarget.str(), chAABB);
				nextRound.push_back(task);
			}
		}

		depth++;
		work = nextRound;
	}

	{ // print number of processed points
		float percentage = float(numPoints - unprocessedPoints) / float(numPoints);
		cout << processedPoints << " of " << numPoints << " points were processed - ";
		cout << int(percentage*100) << "%" << endl;
	}


	stringstream ssCloudJs;
	string strCloudJs = cloudJs.str();
	ssCloudJs << strCloudJs.erase(strCloudJs.find_last_of(",")) << endl;
	ssCloudJs << "\t]" << endl;
	ssCloudJs << "}" << endl;

	ofstream sCloudJs(workDir + "/cloud.js", ios::out);
	sCloudJs << ssCloudJs.str();
	sCloudJs.close();

	delete[] buffer;

	//float addAncestorDuration = float(addAncestorTime)/1000.0f;
	//float addSourceDuration = float(addSourceTime)/1000.0f;
	//float saveCloudJSDuration = float(saveCloudJSTime)/1000.0f;
	//cout << "addAncestorDuration: " << addAncestorDuration << "s" << endl;
	//cout << "addSourceDuration: " << addSourceDuration << "s" << endl;
	//cout << "saveCloudJSDuration: " << saveCloudJSDuration << "s" << endl;
}

void PotreeConverter::addAncestorsToGrid(SparseGrid & grid, string &target){
	auto start = high_resolution_clock::now();
	
	int pos = target.find_last_of("r");
	int numAncestors = target.size() - pos;
	string ancestor = target;
	for(int i = 0; i < numAncestors; i++){
	
		if(boost::filesystem::exists(ancestor + getOutputExtension())){
			//cout << "read ancestor: " << (ancestor + ".las") << endl;
			//LASPointReader *reader = new LASPointReader(ancestor + ".las");
			LASreadOpener opener;
			opener.set_file_name((ancestor + getOutputExtension()).c_str());
			LASreader *reader = opener.open();
			while(reader->read_point()){
				LASpoint &lp = reader->point;
				Point p(lp.get_x(), lp.get_y(), lp.get_z());
				if(aabb.isInside(p)){
					grid.addWithoutCheck(p);
				}
			}
	
			reader->close();
			delete reader;
		}
	
		ancestor = ancestor.substr(0, ancestor.size()-1);
	}
	auto end = high_resolution_clock::now();
	addAncestorTime += duration_cast<milliseconds>(end-start).count();
}

/**
 * 1. load all points from upper levels (r, rx, rxy, until r...depth) and add them to the grid 
 * 2. load points from source and add them to the grid
 * 3. accept all points from source which have a minimum distance of minGap / 2^depth to all 
 *    points inside the grid
 * 4. save accepted points of source to target + index and the remaining points to source + index
 *
 * @returns the indices of the nodes that have been created 
 *  if nodes [r11, r13, r17] were created, then indices [1,3,7] will be returned
 *
 **/
ProcessResult PotreeConverter::process(string source, string target, AABB aabb, int depth){
	vector<int> indices;
	uint64_t numAccepted = 0;
	uint64_t numRejected = 0;

	cout << "processing " << source << endl;
	
	// // print infos
	//	stringstream ssName;
	//	ssName << source.substr(source.find_last_of("d"), source.size() - source.find_last_of("d"));
	//	stringstream ssAABB;
	//	ssAABB << workDir << "/aabb/" << ssName.str();
	//	ofstream fAABB(ssAABB.str(), ios::out);
	//	fAABB << "min: " << aabb.min << endl;
	//	fAABB << "max: " << aabb.max << endl;
	//	fAABB.close();
	//

	SparseGrid grid(aabb, minGap / pow(2.0, depth));

	addAncestorsToGrid(grid, target);

	{ // add source to the grid
		//cout << "add source: " << source << ".las" << endl;
		auto start = high_resolution_clock::now();

		//LASPointReader *reader = new LASPointReader(source + ".las");
		LASreadOpener opener;
		opener.set_file_name((source + getOutputExtension()).c_str());
		LASreader *reader = opener.open();
		LASheader const &header = reader->header;

		LASheader dHeader;
		dHeader.point_data_format = header.point_data_format;
		dHeader.point_data_record_length = header.point_data_record_length;
		dHeader.x_scale_factor = header.x_scale_factor;
		dHeader.y_scale_factor = header.y_scale_factor;
		dHeader.z_scale_factor = header.z_scale_factor;
		dHeader.min_x = header.min_x;
		dHeader.min_y = header.min_y;
		dHeader.min_z = header.min_z;
		dHeader.max_x = header.max_x;
		dHeader.max_y = header.max_y;
		dHeader.max_z = header.max_z;
		dHeader.x_offset = header.x_offset;
		dHeader.y_offset = header.y_offset;
		dHeader.z_offset = header.z_offset;

		LASheader rHeader;
		rHeader.point_data_format = header.point_data_format;
		rHeader.point_data_record_length = header.point_data_record_length;
		rHeader.x_scale_factor = header.x_scale_factor;
		rHeader.y_scale_factor = header.y_scale_factor;
		rHeader.z_scale_factor = header.z_scale_factor;
		rHeader.min_x = header.min_x;
		rHeader.min_y = header.min_y;
		rHeader.min_z = header.min_z;
		rHeader.max_x = header.max_x;
		rHeader.max_y = header.max_y;
		rHeader.max_z = header.max_z;
		rHeader.x_offset = header.x_offset;
		rHeader.y_offset = header.y_offset;
		rHeader.z_offset = header.z_offset;

		vector<LASwriter*> lrOut;
		vector<LASwriter*> ldOut;
		vector<uint64_t> numPoints;
		vector<uint64_t> numPointsRejected;
		vector<AABB> rAABB;
		vector<AABB> dAABB;
		numPoints.resize(8);
		numPointsRejected.resize(8);
		for(int i = 0; i < 8; i++){
			stringstream ssr, ssd;
			ssr << target << i << getOutputExtension();
			ssd << source << i << getOutputExtension();

			rAABB.push_back(AABB());
			dAABB.push_back(AABB());

			try{
				LASwriteOpener opener;
				opener.set_file_name(ssr.str().c_str());
				LASwriter *writer = opener.open(&rHeader);
				lrOut.push_back(writer);
			}catch(...){
				cout << "ssr: " << ssr.str() << endl;
				cout << "index: " << i << endl;
			}

			try{
				LASwriteOpener opener;
				opener.set_file_name(ssd.str().c_str());
				LASwriter *writer = opener.open(&dHeader);
				ldOut.push_back(writer);
			}catch(...){
				cout << "ssd: " << ssd.str() << endl;
				cout << "index: " << i << endl;
			}
		}
		
		while(reader->read_point()){
			LASpoint &lp = reader->point;
			Point p(lp.get_x(), lp.get_y(), lp.get_z());

			bool accepted = grid.add(p);
			int index = nodeIndex(aabb, p);
			if(index == -1){
				continue;
			}

			if(accepted){
				lrOut[index]->write_point(&lp);
				//rAABB[index].update(coord);

				numPoints[index]++;
				numAccepted++;
			}else{
				ldOut[index]->write_point(&lp);
				//dAABB[index].update(coord);

				numPointsRejected[index]++;
				numRejected++;
			}
		}

		reader->close();
		delete reader;

		for(int i = 0; i < 8; i++){

			// close outstreams
			lrOut[i]->close();
			ldOut[i]->close();
			delete lrOut[i];
			delete ldOut[i];
			
			// remove empty files
			stringstream ssr, ssd;
			ssr << target << i;
			ssd << source << i;
			if(numPoints[i] == 0){
				remove((ssr.str() + getOutputExtension()).c_str());
			}
			if(numPointsRejected[i] == 0){
				remove((ssd.str() + getOutputExtension()).c_str());
			}else{
				indices.push_back(i);
			}
		}

		auto end = high_resolution_clock::now();
		addSourceTime += duration_cast<milliseconds>(end-start).count();
		
		{ // update cloud.js
			auto start = high_resolution_clock::now();
		
			for(int i = 0; i < 8; i++){
				if(numPoints[i] > 0){
					stringstream ssName;
					ssName << target.substr(target.find_last_of("r"), target.size() - target.find_last_of("r")) << i;
					cloudJs << "\t\t" << "[\"" << ssName.str() << "\", " << numPoints[i] << "],";
					cloudJs << endl;
				}
			}
			saveCloudJS();
		}
	}

	return ProcessResult(indices, numAccepted, numRejected);
}


void PotreeConverter::saveCloudJS(){
	stringstream ssCloudJs;
	string strCloudJs = cloudJs.str();
	ssCloudJs << strCloudJs.erase(strCloudJs.find_last_of(",")) << endl;
	ssCloudJs << "\t]" << endl;
	ssCloudJs << "}" << endl;

	ofstream sCloudJs(workDir + "/cloud.js", ios::out);
	sCloudJs << ssCloudJs.str();
	sCloudJs.close();
}