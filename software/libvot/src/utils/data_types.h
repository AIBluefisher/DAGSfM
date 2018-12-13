
/** \file data_types.h
 *	\brief internal data types (structs and classes) used in libvot
 */
#ifndef VOT_DATA_TYPES_H
#define VOT_DATA_TYPES_H

#include <cstdlib>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <cmath>
#include <cassert>

#include "global_params.h"

namespace vot
{
/** @brief Sift data structure used in libvot */
class SiftData
{
public:
	SiftData()
	{
		name_ = 'S' + ('I' << 8) + ('F' << 16) + ('T' << 24);
		version_ = 'V' + ('5' << 8) + ('.' << 16) + ('0' << 24);
		npoint_ = 0;
		nLocDim_ = 5;
		nDesDim_ = 128;
		dp_ = nullptr; lp_ = nullptr;
	}

	// copy constructor
	SiftData(const SiftData & obj):
	    name_(obj.name_), version_(obj.version_), npoint_(obj.npoint_),
	    nLocDim_(obj.nLocDim_), nDesDim_(obj.nDesDim_)
	{
		//copy feature descriptors
		dp_ = new DTYPE [obj.npoint_ * nDesDim_];
		memcpy(dp_, obj.dp_, sizeof(DTYPE) * obj.npoint_ * nDesDim_);

		//copy location descriptors
		lp_ = new LTYPE [obj.npoint_ * nLocDim_];
		memcpy(lp_, obj.lp_, sizeof(LTYPE) * obj.npoint_ * nLocDim_);
	}

	// copy assignment operator
	SiftData & operator=(const SiftData &rhs)
	{
		name_ = rhs.name_;
		version_ = rhs.version_;
		npoint_ = rhs.npoint_;
		nLocDim_ = rhs.nLocDim_;
		nDesDim_ = rhs.nDesDim_;

		//copy feature descriptors
		dp_ = new DTYPE [rhs.npoint_ * nDesDim_];
		memcpy(dp_, rhs.dp_, sizeof(DTYPE) * rhs.npoint_ * nDesDim_);

		//copy location descriptors
		lp_ = new LTYPE [rhs.npoint_ * nLocDim_];
		memcpy(lp_, rhs.lp_, sizeof(LTYPE) * rhs.npoint_ * nLocDim_);

		return *this;
	}

	// destructor
	~SiftData()
	{
		if (nullptr != dp_) delete [] dp_;
		if (nullptr != lp_) delete [] lp_;
	}

	//TODO(tianwei): add serialization library for I/O
	///
	/// \brief ReadSiftFile: read sift file, this function is compatible with the sfm version of sift file
	/// \param szFileName: the name of the sift file
	/// \return
	///
	bool ReadSiftFile(std::string const &szFileName)
	{
		name_ = 'S' + ('I' << 8) + ('F' << 16) + ('T' << 24);
		version_ = 'V' + ('5' << 8) + ('.' << 16) + ('0' << 24);
		FILE *fd;
		if ((fd = fopen(szFileName.c_str(), "rb")) == nullptr) {
			std::cerr << "[ReadSiftFile] Can't read sift file " << szFileName << '\n';
			return false;
		}
		int f = fread(&name_, sizeof(int), 1, fd);
		int g = fread(&version_, sizeof(int), 1, fd);
		if (f != 1 || g != 1) {
			std::cerr << "[ReadSiftFile] Reading error\n";
			return false;
		}

		if (name_ == ('S'+ ('I'<<8)+('F'<<16)+('T'<<24))) {
			int a, b, c;
			a = fread(&npoint_, sizeof(int), 1, fd);
			b = fread(&nLocDim_, sizeof(int), 1, fd);
			c = fread(&nDesDim_, sizeof(int), 1, fd);
			if (a != 1 || b != 1 || c!= 1) {
				std::cerr << "[ReadSiftFile] Reading error\n";
				return false;
			}

			lp_ = new LTYPE [npoint_ * nLocDim_];          //restoring location data
			dp_ = new DTYPE [npoint_ * nDesDim_];          //restoring descriptor data
			if (npoint_ >= 0 && nLocDim_ > 0 && nDesDim_ == FDIM) {
				int d, e;
				d = fread(lp_, sizeof(LTYPE), nLocDim_ * npoint_, fd);
				e = fread(dp_, sizeof(DTYPE), nDesDim_ * npoint_, fd);
				if (d != nLocDim_ * npoint_ || e != nDesDim_ * npoint_) {
					std::cerr << "[ReadSiftFile] Reading error\n";
					return false;
				}
				fclose(fd);
			}
			else {
				fclose(fd);
				return false;
			}
		}
		else {
			fclose(fd);
			return false;
		}
		return true;
	}

	///
	/// \brief ConvertChar2Float: Read a char sift file and convert it to DTYPE(float/double) sift.
	///                           The descriptors are L1 normalized and then square-rooted (RootSift)
	/// \param szFileName: the name of the sift file
	/// \return
	///
	bool ReadChar2DTYPE(std::string const &szFileName)
	{
		name_ = 'S' + ('I' << 8) + ('F' << 16) + ('T' << 24);
		version_ = 'V' + ('5' << 8) + ('.' << 16) + ('0' << 24);
		FILE *fd;
		if ((fd = fopen(szFileName.c_str(), "rb")) == nullptr) {
			std::cout << "[ReadSiftFile] Can't read sift file " << szFileName << '\n';
			return false;
		}
		int f = fread(&name_, sizeof(int), 1, fd);
		int g = fread(&version_, sizeof(int), 1, fd);
		if (f != 1 || g != 1) {
			std::cerr << "[ReadSiftFile] Reading error\n";
			return false;
		}

		if (name_ == ('S'+ ('I'<<8)+('F'<<16)+('T'<<24))) {
			int a, b, c;
			a = fread(&npoint_, sizeof(int), 1, fd);
			b = fread(&nLocDim_, sizeof(int), 1, fd);
			c = fread(&nDesDim_, sizeof(int), 1, fd);
			if (a != 1 || b != 1 || c!= 1) {
				std::cerr << "[ReadSiftFile] Reading error\n";
				return false;
			}

			lp_ = new LTYPE [npoint_ * nLocDim_];          //restoring location data
			dp_ = new DTYPE [npoint_ * nDesDim_];          //restoring descriptor data
			unsigned char *temp_dp = new unsigned char [npoint_ * nDesDim_];    // restoring the temporary descriptor data
			if (npoint_ >= 0 && nLocDim_ > 0 && nDesDim_ == FDIM) {
				int d, e;
				d = fread(lp_, sizeof(LTYPE), nLocDim_ * npoint_, fd);
				e = fread(temp_dp, sizeof(unsigned char), nDesDim_ * npoint_, fd);
				if (d != nLocDim_ * npoint_ || e != nDesDim_ * npoint_) {
					std::cerr << "[ReadSiftFile] Reading error\n";
					return false;
				}
				// convert SIFT to RootSIFT
				for (int i = 0; i < npoint_; i++) {
					DTYPE magnitude = 0.0;
					for (int j = 0; j < nDesDim_; j++) {
						magnitude += (DTYPE) temp_dp[i*nDesDim_+j];
					}
					for (int j = 0; j < nDesDim_; j++) {
						dp_[i*nDesDim_+j] = sqrt((double) temp_dp[i*nDesDim_+j] / magnitude);
					}
				}
				delete [] temp_dp;
				fclose(fd);
			}
			else {
				fclose(fd);
				return false;
			}
		}
		else {
			fclose(fd);
			return false;
		}
		return true;
	}

	///
	/// \brief ReadOpenmvgDesc: read desc file of openmvg
	/// \param szFileName: the name of the sift file
	/// \return
	///
	template <typename T, std::size_t N>
	bool ReadOpenmvgDesc(std::string const &szFileName)
	{
		assert(sizeof(T) == sizeof(DTYPE) && N == FDIM &&
		       "Internal and external feature parameters are not consistent\n");

		std::ifstream fileIn(szFileName.c_str(), std::ios::in | std::ios::binary);
		if (!fileIn.is_open()) {
			std::cerr << "[ReadOpenmvgDesc] Fail to load openmvg desc file.\n";
			return false;
		}
		//Read the number of descriptor in the file
		std::size_t cardDesc = 0;
		fileIn.read((char*) &cardDesc, sizeof(std::size_t));
		npoint_ = cardDesc;

		//TODO(tianwei): restoring location data (since it's useless for now, fill it with 0)
		lp_ = new LTYPE [npoint_ * nLocDim_];
		for (size_t i = 0; i < npoint_ * nLocDim_; i++)
			lp_[i] = 0;

		//restoring descriptor data
		dp_ = new DTYPE [npoint_ * nDesDim_];
		fileIn.read((char*)dp_, N*sizeof(T)*npoint_);

		bool bOk = !fileIn.bad();
		fileIn.close();

		return bOk;
	}

	bool SaveSiftFile(std::string const &szFileName)
	{
		FILE *fd;
		if ((fd = fopen(szFileName.c_str(), "wb")) == nullptr) {
			std::cerr << "[SaveSiftFile] Can't open file " << szFileName << " for saving\n";
			return false;
		}
		int f = fwrite(&name_, sizeof(int), 1, fd);
		int g = fwrite(&version_, sizeof(int), 1, fd);
		if (f != 1 || g != 1) {
			std::cerr << "[SaveSiftFile] Writing error\n";
			return false;
		}

		if (name_ == ('S'+ ('I'<<8)+('F'<<16)+('T'<<24))) {
			int a, b, c;
			a = fwrite(&npoint_, sizeof(int), 1, fd);
			b = fwrite(&nLocDim_, sizeof(int), 1, fd);
			c = fwrite(&nDesDim_, sizeof(int), 1, fd);
			if (a != 1 || b != 1 || c!= 1) {
				std::cerr << "[SaveSiftFile] Write error\n";
				return false;
			}

			if (npoint_ >= 0 && nLocDim_ > 0 && nDesDim_ == FDIM) {
				int d, e;
				d = fwrite((void*) lp_, sizeof(LTYPE), nLocDim_ * npoint_, fd);
				e = fwrite((void*) dp_, sizeof(DTYPE), nDesDim_ * npoint_, fd);
				if (d != nLocDim_ * npoint_ || e != nDesDim_ * npoint_) {
					std::cerr << "[SaveSiftFile] Writing error\n";
					return false;
				}
				fclose(fd);
			}
			else {
				fclose(fd);
				return false;
			}
		}
		else {
			fclose(fd);
			return false;
		}
		return true;
	}

	const int getFeatureNum() const {return npoint_;}
	void setFeatureNum(int feat_num) {npoint_ = feat_num;}
	const int getLocDim() const {return nLocDim_;}
	void setLocDim(int loc_dim) {nLocDim_ = loc_dim;}
	const int getDesDim() const {return nDesDim_;}
	void setDesDim(int des_dim) {nDesDim_ = des_dim;}
	const int getVersion() const {return version_;}
	void setVersion(int version) {version_ = version;}

	void clear()
	{
		if (nullptr != dp_) delete [] dp_;
		if (nullptr != lp_) delete [] lp_;
		dp_ = nullptr; lp_ = nullptr;
	}

	// To manipulate data in SiftData
	LTYPE* &getLocPointer() {return lp_;}
	DTYPE* &getDesPointer() {return dp_;}

	/**
	 * @brief showDesc: output a single descriptor, a debugging function
	 * @param i: the index of descriptor
	 */
	void showDesc(int i)
	{
		if (i >= npoint_) {
			return;
		}
		else {
			std::cout << "[";
			for(int j = 0; j < FDIM; j++)
			{
				std::cout << (int)dp_[i*FDIM + j] << " ";
			}
			std::cout << "]\n";
		}
	}

private:
	int name_;
	int version_;
	int npoint_;
	int nLocDim_;
	int nDesDim_;
	DTYPE *dp_;
	LTYPE *lp_;
};

}   // end of namespace vot

#endif	// VOT_DATA_TYPES_H
