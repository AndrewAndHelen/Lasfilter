#include "LasFilter.h"

//Use elevation interval division method to eliminate gross errors
//First, divide the point cloud into intervals according to a certain elevation interval;
//Then respectively start from the uppermost interval and the lowermost interval to judge in the middle, if the number of points in the interval is less than the threshold and the interval before the interval is marked as gross error interval
bool LasFilter::deltGross(std::string _szLasIn,
	std::string _szLasOut,
	double _lfLdrPtInterv)
{
	int ptnum_gross_cube_low;//The laser points in the grid are less than the threshold, and the points in the grid are regarded as gross points.
	int ptnum_gross_cube_high;//Because the grid scale will become smaller during the iteration process, the threshold should also be appropriately reduced
	int ptnum_gross_cube_cur;

	double cube_size_low_xy;
	double cube_size_high_xy;
	double cube_size_cur_xy;
	double cube_size_low_z; 
	double cube_size_high_z;
	double cube_size_cur_z;

	cube_size_low_xy = 3 * _lfLdrPtInterv;

	ptnum_gross_cube_low = 5; //The laser points in the grid are less than the threshold, then the points in the grid are regarded as gross points
	ptnum_gross_cube_high = 70; //Since the grid scale will become smaller during the iteration, the threshold should also be appropriately reduced

	//Considering the different density of point clouds in the xy direction and z direction, grid sizes of different scales should be used
	cube_size_high_xy = 40;
	cube_size_low_z = 1; 
	cube_size_high_z = 30;

	int iter_num = 3;

	// open input file
	LASreadOpener lasreadopener;
	LASheader las_header_read;
	LASreader* lasreader;

	lasreadopener.set_file_name(_szLasIn.c_str());

	if (!lasreadopener.active()) {
		std::cout << "ERROR: no input specified" << std::endl;
		return false;
	}

	lasreader = lasreadopener.open();
	if (!lasreader) {
		std::cerr << "ERROR: could not open LAS file: " << _szLasIn << std::endl;
		return false;
	}

	las_header_read = lasreader->header;
	las_header_read.user_data_after_header = nullptr;

	int npoints = lasreader->npoints;
	bool *bSurviveFlag = new bool[npoints];
	for (int k = 0; k < npoints; k++) { bSurviveFlag[k] = true; } //Considering the different density of point clouds in the xy direction and z direction, grid sizes of different scales should be used

	//////////////////////////////////////////////////////////////////////////
	//Calculate 3D grid
	double xmin, xmax, ymin, ymax, zmin, zmax;
	xmin = lasreader->header.min_x;
	xmax = lasreader->header.max_x;
	ymin = lasreader->header.min_y;
	ymax = lasreader->header.max_y;
	zmin = lasreader->header.min_z;
	zmax = lasreader->header.max_z;

	lasreader->close();
	lasreader = NULL;

	//Judge which points are gross handicap points, and use loop iteration algorithm
	for (int iter = 0; iter < iter_num + 1; iter++)
	{
		ptnum_gross_cube_cur = ptnum_gross_cube_high - iter * ((ptnum_gross_cube_high - ptnum_gross_cube_low) / iter_num);
		if (ptnum_gross_cube_cur < ptnum_gross_cube_low) { ptnum_gross_cube_cur = ptnum_gross_cube_low; }

		cube_size_cur_xy = cube_size_high_xy - iter * ((cube_size_high_xy - cube_size_low_xy) / iter_num);
		if (cube_size_cur_xy < cube_size_low_xy) { cube_size_cur_xy = cube_size_low_xy; }

		cube_size_cur_z = cube_size_high_z - iter * ((cube_size_high_z - cube_size_low_z) / iter_num);
		if (cube_size_cur_z < cube_size_low_z) { cube_size_cur_z = cube_size_low_z; }

		//////////////////////////////////////////////////////////////////////////
		// open input file
		lasreadopener.set_file_name(_szLasIn.c_str());

		if (!lasreadopener.active()) {
			std::cout << "ERROR: no input specified" << std::endl;
			return false;
		}

		lasreader = lasreadopener.open();
		if (lasreader == 0) {
			std::cout << "ERROR: could not open lasreader" << std::endl;
			return false;
		}

		las_header_read = lasreader->header;
		las_header_read.user_data_after_header = nullptr;

		U8 point_type = las_header_read.point_data_format;
		U16 point_size = las_header_read.point_data_record_length;

		LASpoint point;
		point.init(&las_header_read, point_type, point_size, &las_header_read);

		//	
		int nXSz, nYSz, nZSz;
		nXSz = int((xmax - xmin) / cube_size_cur_xy) + 1;//
		nYSz = int((ymax - ymin) / cube_size_cur_xy) + 1;
		nZSz = int((zmax - zmin) / cube_size_cur_z) + 1;
		//Apply for an array to store the number of laser points in the rectangle
		int nCubeNum = nXSz * nYSz*nZSz;
		unsigned char *pPtNoInCube = new unsigned char[nCubeNum];

		if (!pPtNoInCube)
		{
			return 0;
		}

		bool *pCubeSrvieFlag = new bool[nCubeNum]; //Determine whether the points in each grid are retained: true retain false remove
		if (!pCubeSrvieFlag)
		{
			return 0;
		}
		for (int i = 0; i < nCubeNum; i++)
		{
			pCubeSrvieFlag[i] = true;
		}
		for (int i = 0; i < nCubeNum; i++)
		{
			pPtNoInCube[i] = 0;
		}

		int nCount = 0;
		while (lasreader->read_point()) {
			nCount++;
			point = lasreader->point;

			//Points judged as gross errors in the previous iteration will not participate in this statistics
			if (!bSurviveFlag[nCount - 1]) { continue; }
			//Determine which grid the point falls into
			int xc = int((point.get_x() - xmin) / cube_size_cur_xy); 
			int yc = int((point.get_y() - ymin) / cube_size_cur_xy); 
			int zc = int((point.get_z() - zmin) / cube_size_cur_z); 
			if (pPtNoInCube[zc*nXSz*nYSz + yc * nXSz + xc] > 254)
			{
				pPtNoInCube[zc*nXSz*nYSz + yc * nXSz + xc] = 255;
			}
			else
			{
				pPtNoInCube[zc*nXSz*nYSz + yc * nXSz + xc]++;
			}

		}

		lasreader->close();
		lasreader = NULL;

		//Start to judge whether the points in the grid should be eliminated as gross errors
		//Make the first judgment
		for (int iz = 0; iz < nZSz; iz++)
		{
			for (int iy = 0; iy < nYSz; iy++)
			{
				for (int ix = 0; ix < nXSz; ix++)
				{
					int nCubeCur = iz * nXSz*nYSz + iy * nXSz + ix;

					if (pPtNoInCube[nCubeCur] < ptnum_gross_cube_cur) 
					{
						pCubeSrvieFlag[nCubeCur] = false;

						bool survflag = false;
						int ix_s, iy_s, iz_s;
						ix_s = ix - 1;
						iy_s = iy - 1;
						iz_s = iz - 1;
						for (int t1 = 0; t1 < 3; t1++)
						{
							for (int t2 = 0; t2 < 3; t2++)
							{
								for (int t3 = 0; t3 < 3; t3++)
								{
									int ix_t, iy_t, iz_t;
									ix_t = ix_s + t3;
									iy_t = iy_s + t2;
									iz_t = iz_s + t1;
									if (ix_t < 0 || ix_t > nXSz - 1)
									{
										continue;
									}
									if (iy_t < 0 || iy_t > nYSz - 1)
									{
										continue;
									}
									if (iz_t < 0 || iz_t > nZSz - 1)
									{
										continue;
									}
									//
									int nCubeTemp = iz_t * nXSz*nYSz + iy_t * nXSz + ix_t;
									//
									if (nCubeTemp < 0 || nCubeTemp > nCubeNum - 1 || nCubeTemp == nCubeCur)
									{
										continue;
									}
									if (pPtNoInCube[nCubeTemp] > ptnum_gross_cube_cur)
									{
										survflag = true;//As long as the number of points in a grid in its 26 neighborhood is greater than the threshold, the current grid will be retained
										t1 = 3; t2 = 3; t3 = 3;
									}
								}
							}
						}

						if (survflag)//If the number of points in all the grids in the 26 neighborhood is less than the threshold, the grid is still marked as deleted, otherwise it is marked as reserved
						{
							pCubeSrvieFlag[nCubeCur] = true;
						}
					}
					else
					{
						pCubeSrvieFlag[nCubeCur] = true;
					}
				}
			}
		}
		delete[] pPtNoInCube; pPtNoInCube = NULL;

		//Mark the gross handicap
		//Reopen the file
		lasreadopener.set_file_name(_szLasIn.c_str());

		if (!lasreadopener.active()) {
			std::cout << "ERROR: no input specified" << std::endl;
			return false;
		}

		lasreader = lasreadopener.open();
		if (!lasreader) {
			std::cerr << "ERROR: could not open LAS file: " << _szLasIn << std::endl;
			return false;
		}

		las_header_read = lasreader->header;
		las_header_read.user_data_after_header = nullptr;

		point_type = las_header_read.point_data_format;
		point_size = las_header_read.point_data_record_length;

		point.init(&las_header_read, point_type, point_size, &las_header_read);
		
		nCount = 0;
		while (lasreader->read_point())
		{
			nCount++;
			point = lasreader->point;

			if (!bSurviveFlag[nCount - 1]) { continue; }

			long xc = int((point.get_x() - xmin) / cube_size_cur_xy); 
			long yc = int((point.get_y() - ymin) / cube_size_cur_xy); 
			long zc = int((point.get_z() - zmin) / cube_size_cur_z); 

			int nCurCube = zc * nXSz*nYSz + yc * nXSz + xc;

			if (!pCubeSrvieFlag[nCurCube])
			{
				bSurviveFlag[nCount - 1] = 0;
			}
		}

		lasreader->close();
		lasreader = NULL;
		delete[] pCubeSrvieFlag; pCubeSrvieFlag = NULL;
		//////////////////////////////////////////////////////////////////////////
		if (cube_size_cur_xy <= cube_size_low_xy)
		{
			break;
		}
	}

	lasreadopener.set_file_name(_szLasIn.c_str());

	if (!lasreadopener.active()) {
		std::cout << "ERROR: no input specified" << std::endl;
		return false;
	}

	lasreader = lasreadopener.open();
	if (!lasreader) {
		std::cerr << "ERROR: could not open LAS file: " << _szLasIn << std::endl;
		return false;
	}

	las_header_read = lasreader->header;
	las_header_read.user_data_after_header = nullptr;

	U8 point_type = las_header_read.point_data_format;
	U16 point_size = las_header_read.point_data_record_length;
	//
	int surviving_number_of_point_records = 0;
	unsigned int surviving_number_of_points_by_return[] = { 0,0,0,0,0,0,0,0 };
	bool first_surviving_point = true;
	LASpoint surviving_point_min;
	LASpoint surviving_point_max;
	double surviving_gps_time_min = 0;
	double surviving_gps_time_max = 0;
	short surviving_rgb_min[3] = { 0,0,0 };
	short surviving_rgb_max[3] = { 0,0,0 };
	int scale_rgb = 0;

	double xyz_min[3];
	double xyz_max[3];

	LASpoint point;
	point.init(&las_header_read, point_type, point_size, &las_header_read);

	int nCount = 0;
	while (lasreader->read_point())
	{
		nCount++;
		point = lasreader->point;

		//Compare the elevation of the point with the maximum elevation of the grid
		if (bSurviveFlag[nCount - 1])
		{
			if (surviving_number_of_point_records == 0)
			{
				xyz_min[0] = point.get_x();
				xyz_min[1] = point.get_y();
				xyz_min[2] = point.get_z();
				xyz_max[0] = point.get_x();
				xyz_max[1] = point.get_y();
				xyz_max[2] = point.get_z();
			}
			else
			{
				if (point.get_x() < xyz_min[0])
				{
					xyz_min[0] = point.get_x();
				}
				if (point.get_y() < xyz_min[1])
				{
					xyz_min[1] = point.get_y();
				}
				if (point.get_z() < xyz_min[2])
				{
					xyz_min[2] = point.get_z();
				}
				if (point.get_x() > xyz_max[0])
				{
					xyz_max[0] = point.get_x();
				}
				if (point.get_y() > xyz_max[1])
				{
					xyz_max[1] = point.get_y();
				}
				if (point.get_z() > xyz_max[2])
				{
					xyz_max[2] = point.get_z();
				}
			}
			surviving_number_of_point_records++;
			if (point.return_number - 1 >= 0 && point.return_number - 1 < 8)
			{
				surviving_number_of_points_by_return[point.return_number - 1]++;
			}
			if (first_surviving_point)//The first point to be recorded
			{//At this time, the LASpoint = overload method just assigns the X, Y, Z (I32) of the copied point to the copy point, without offset and zoom factor
				surviving_point_min = point; 
				surviving_point_max = point; 

				first_surviving_point = false;
			}
			if (point.have_rgb)
			{
				if (scale_rgb == 1)
				{
					point.rgb[0] = (point.rgb[0] / 256);
					point.rgb[1] = (point.rgb[1] / 256);
					point.rgb[2] = (point.rgb[2] / 256);
				}
				else if (scale_rgb == 2)
				{
					point.rgb[0] = (point.rgb[0] * 256);
					point.rgb[1] = (point.rgb[1] * 256);
					point.rgb[2] = (point.rgb[2] * 256);
				}
				surviving_rgb_min[0] = point.rgb[0];
				surviving_rgb_min[1] = point.rgb[1];
				surviving_rgb_min[2] = point.rgb[2];
				surviving_rgb_max[0] = point.rgb[0];
				surviving_rgb_max[1] = point.rgb[1];
				surviving_rgb_max[2] = point.rgb[2];
			}
			first_surviving_point = false;
		}
		else
		{
			if (point.get_X() < surviving_point_min.X) 
			{
				surviving_point_min.X = point.get_X();
			}
			else if (point.get_X() > surviving_point_max.X) 
			{
				surviving_point_max.X = point.get_X();
			}
			if (point.get_Y() < surviving_point_min.Y)
			{
				surviving_point_min.Y = point.get_Y();
			}
			else if (point.get_Y() > surviving_point_max.Y) 
			{
				surviving_point_max.Y = point.get_Y();
			}
			if (point.get_Z() < surviving_point_min.Z)
			{
				surviving_point_min.Z = point.get_Z();
			}
			else if (point.get_Z() > surviving_point_max.Z)
			{
				surviving_point_max.Z = point.get_Z();
			}
		
		//	if (point.intensity < surviving_point_min.intensity) //�ҵ�ǿ����Сֵ
		//	{
		//		surviving_point_min.intensity = point.intensity;
		//	}
		//	else if (point.intensity > surviving_point_max.intensity) //�ҵ�ǿ�����ֵ
		//	{
		//		surviving_point_max.intensity = point.intensity;
		//	}
		//	if (point.edge_of_flight_line < surviving_point_min.edge_of_flight_line)
		//	{
		//		surviving_point_min.edge_of_flight_line = point.edge_of_flight_line;
		//	}
		//	else if (point.edge_of_flight_line > surviving_point_max.edge_of_flight_line)
		//	{
		//		surviving_point_max.edge_of_flight_line = point.edge_of_flight_line;
		//	}
		//	if (point.scan_direction_flag < surviving_point_min.scan_direction_flag)
		//	{
		//		surviving_point_min.scan_direction_flag = point.scan_direction_flag;
		//	}
		//	else if (point.scan_direction_flag > surviving_point_max.scan_direction_flag)
		//	{
		//		surviving_point_max.scan_direction_flag = point.scan_direction_flag;
		//	}
		//	if (point.return_number < surviving_point_min.return_number)
		//	{
		//		surviving_point_min.return_number = point.return_number;
		//	}
		//	else if (point.return_number > surviving_point_max.return_number)
		//	{
		//		surviving_point_max.return_number = point.return_number;
		//	}
		//	if (point.classification < surviving_point_min.classification)
		//	{
		//		surviving_point_min.classification = point.classification;
		//	}
		//	else if (point.classification > surviving_point_max.classification)
		//	{
		//		surviving_point_max.classification = point.classification;
		//	}
		//	if (point.scan_angle_rank < surviving_point_min.scan_angle_rank)
		//	{
		//		surviving_point_min.scan_angle_rank = point.scan_angle_rank;
		//	}
		//	else if (point.scan_angle_rank > surviving_point_max.scan_angle_rank)
		//	{
		//		surviving_point_max.scan_angle_rank = point.scan_angle_rank;
		//	}
		//	if (point.user_data < surviving_point_min.user_data)
		//	{
		//		surviving_point_min.user_data = point.user_data;
		//	}
		//	else if (point.user_data > surviving_point_max.user_data)
		//	{
		//		surviving_point_max.user_data = point.user_data;
		//	}
		//	if (point.point_source_ID < surviving_point_min.point_source_ID)
		//	{
		//		surviving_point_min.point_source_ID = point.point_source_ID;
		//	}
		//	else if (lasreader->point.point_source_ID > surviving_point_max.point_source_ID)
		//	{
		//		surviving_point_max.point_source_ID = lasreader->point.point_source_ID;
		//	}
		//	if (lasreader->point.point_source_ID < surviving_point_min.point_source_ID)
		//	{
		//		surviving_point_min.point_source_ID = lasreader->point.point_source_ID;
		//	}
		//	else if (lasreader->point.point_source_ID > surviving_point_max.point_source_ID)
		//	{
		//		surviving_point_max.point_source_ID = lasreader->point.point_source_ID;
		//	}
		//	if (point.have_gps_time)
		//	{
		//		if (point.get_gps_time() < surviving_gps_time_min)
		//		{
		//			surviving_gps_time_min = point.get_gps_time();
		//		}
		//		else if (point.get_gps_time() > surviving_gps_time_max)
		//		{
		//			surviving_gps_time_max = point.get_gps_time();
		//		}
		//	}
		//	if (point.have_rgb)
		//	{
		//		if (scale_rgb == 1)
		//		{
		//			point.rgb[0] = (point.rgb[0] / 256);
		//			point.rgb[1] = (point.rgb[1] / 256);
		//			point.rgb[2] = (point.rgb[2] / 256);
		//		}
		//		else if (scale_rgb == 2)
		//		{
		//			point.rgb[0] = (point.rgb[0] * 256);
		//			point.rgb[1] = (point.rgb[1] * 256);
		//			point.rgb[2] = (point.rgb[2] * 256);
		//		}
		//		surviving_rgb_min[0] = point.rgb[0];
		//		surviving_rgb_min[1] = point.rgb[1];
		//		surviving_rgb_min[2] = point.rgb[2];
		//		surviving_rgb_max[0] = point.rgb[0];
		//		surviving_rgb_max[1] = point.rgb[1];
		//		surviving_rgb_max[2] = point.rgb[2];
		//	}
		//	if (point.rgb[0] < surviving_rgb_min[0])
		//	{
		//		surviving_rgb_min[0] = point.rgb[0];
		//	}
		//	else if (point.rgb[0] > surviving_rgb_max[0])
		//	{
		//		surviving_rgb_max[0] = point.rgb[0];
		//	}
		//	if (point.rgb[1] < surviving_rgb_min[1])
		//	{
		//		surviving_rgb_min[1] = point.rgb[1];
		//	}
		//	else if (point.rgb[1] > surviving_rgb_max[1])
		//	{
		//		surviving_rgb_max[1] = point.rgb[1];
		//	}
		//	if (point.rgb[2] < surviving_rgb_min[2])
		//	{
		//		surviving_rgb_min[2] = point.rgb[2];
		//	}
		//	else if (point.rgb[2] > surviving_rgb_max[2])
		//	{
		//		surviving_rgb_max[2] = point.rgb[2];
		//	}
		//		
		}

	}
	lasreader->close();
	lasreader = NULL;

	lasreadopener.set_file_name(_szLasIn.c_str());

	if (!lasreadopener.active()) {
		std::cout << "ERROR: no input specified" << std::endl;
		return false;
	}

	lasreader = lasreadopener.open();
	if (!lasreader) {
		std::cerr << "ERROR: could not open LAS file: " << _szLasIn << std::endl;
		return false;
	}

	las_header_read = lasreader->header;
	las_header_read.user_data_after_header = nullptr;

	point_type = las_header_read.point_data_format;
	point_size = las_header_read.point_data_record_length;

	LASwriteOpener laswriteopener;
	LASheader            las_header_write;
	LASwriter*           laswriter;
	laswriteopener.set_file_name(_szLasOut.c_str());

	if (!laswriteopener.active()) {
		std::cerr << "error: could not write las file: " << _szLasOut << std::endl;
		exit(-1);
	}

	if (strcmp(&_szLasOut.back(), "z") == 0) {
		laswriteopener.set_format(LAS_TOOLS_FORMAT_LAZ);
	}
	else {
		laswriteopener.set_format(LAS_TOOLS_FORMAT_LAS);
	}
	las_header_write.x_offset = las_header_read.x_offset;
	las_header_write.y_offset = las_header_read.y_offset;
	las_header_write.z_offset = las_header_read.z_offset;
	las_header_write.x_scale_factor = las_header_read.x_scale_factor;
	las_header_write.y_scale_factor = las_header_read.y_scale_factor;
	las_header_write.z_scale_factor = las_header_read.z_scale_factor;

	for(int i=0;i<4;i++)
		las_header_write.file_signature[i] = las_header_read.file_signature[i];

	las_header_write.file_source_ID = las_header_read.file_source_ID;
	las_header_write.global_encoding = las_header_read.global_encoding;
	las_header_write.project_ID_GUID_data_1 = las_header_read.project_ID_GUID_data_1;
	las_header_write.project_ID_GUID_data_2 = las_header_read.project_ID_GUID_data_2;
	las_header_write.project_ID_GUID_data_3 = las_header_read.project_ID_GUID_data_3;
	for (int i = 0; i < 8; i++)
		las_header_write.project_ID_GUID_data_4[i] = las_header_read.project_ID_GUID_data_4[i];

	las_header_write.version_major = las_header_read.version_major; // 1
	las_header_write.version_minor = las_header_read.version_minor; // 2
	for (int i = 0; i < 32; i++)
	{
		las_header_write.system_identifier[i] = las_header_read.system_identifier[i];
		las_header_write.generating_software[i] = las_header_read.generating_software[i];
	}
	las_header_write.file_creation_day = las_header_read.file_creation_day;
	las_header_write.file_creation_year = las_header_read.file_creation_year;

	las_header_write.point_data_format = las_header_read.point_data_format;
	las_header_write.point_data_record_length = las_header_read.point_data_record_length;

	laswriter = laswriteopener.open(&las_header_write);
	if (!laswriter) {
		return false;
	}

	LASpoint laspoint_w;
	if (!laspoint_w.init(&las_header_write, las_header_write.point_data_format, las_header_write.point_data_record_length, 0)) {
		return false;
	}

	nCount = 0;
	while (lasreader->read_point())
	{
		nCount++;
		if (!bSurviveFlag[nCount - 1])
		{
			continue;
		}
		// write the first point
		laswriter->write_point(&lasreader->point);
		laswriter->update_inventory(&lasreader->point);
	}
	delete[] bSurviveFlag; bSurviveFlag = NULL;

	las_header_write.number_of_point_records = surviving_number_of_point_records;
	for (int i = 0; i < 5; i++)
		las_header_write.number_of_points_by_return[i] = surviving_number_of_points_by_return[i];

	las_header_write.min_x = surviving_point_min.X*las_header_write.x_scale_factor + las_header_write.x_offset;
	las_header_write.max_x = surviving_point_max.X*las_header_write.x_scale_factor + las_header_write.x_offset;
	las_header_write.min_y = surviving_point_min.Y*las_header_write.y_scale_factor + las_header_write.y_offset;
	las_header_write.max_y = surviving_point_max.Y*las_header_write.y_scale_factor + las_header_write.y_offset;
	las_header_write.min_z = surviving_point_min.Z*las_header_write.z_scale_factor + las_header_write.z_offset;
	las_header_write.max_z = surviving_point_max.Z*las_header_write.z_scale_factor + las_header_write.z_offset;

	laswriter->update_header(&las_header_write, true);

	lasreader->close();
	lasreader = NULL;

	laswriter->close();
	laswriter = NULL;

	return true;
}

bool LasFilter::gridFilter(std::string _szLasIn, std::string _szLasOut, double lfDet, double hight_thresh)
{
	//lfDet = 0.8; //The dot spacing is 1.3 times the grid size
	//hight_thresh = 0.25; //The point where the difference between the elevation value and the highest value in the grid exceeds the threshold is considered as a wall point

	// open input file
	LASreadOpener lasreadopener;
	LASheader las_header_read;
	LASreader* lasreader;

	lasreadopener.set_file_name(_szLasIn.c_str());

	if (!lasreadopener.active()) {
		std::cout << "ERROR: no input specified" << std::endl;
		return false;
	}

	lasreader = lasreadopener.open();
	if (!lasreader) {
		std::cerr << "ERROR: could not open LAS file: " << _szLasIn << std::endl;
		return false;
	}

	las_header_read = lasreader->header;
	las_header_read.user_data_after_header = nullptr;

	U8 point_type = las_header_read.point_data_format;
	U16 point_size = las_header_read.point_data_record_length;

	int npoints = lasreader->npoints;
	LASpoint point_r;
	point_r.init(&las_header_read, point_type, point_size, &las_header_read);
	//////////////////////////////////////////////////////////////////////////
	//Calculate the width and height of the grid
	//Use two sets of grids that are misaligned by half the grid width to determine
	double xmin, xmax, ymin, ymax, zmin, zmax;
	xmin = lasreader->header.min_x;
	xmax = lasreader->header.max_x;
	ymin = lasreader->header.min_y;
	ymax = lasreader->header.max_y;
	zmin = lasreader->header.min_z;
	zmax = lasreader->header.max_z;

	long nWidth, nHeight;
	nWidth = int((xmax - xmin) / lfDet) + 2;//
	nHeight = int((ymax - ymin) / lfDet) + 2;

	double *pHightIfo_a = new double[nWidth * nHeight];
	double *pHightIfo_b = new double[nWidth * nHeight];
	for (int i = 0; i < nWidth * nHeight; i++)
	{
		pHightIfo_a[i] = -9999.0;
		pHightIfo_b[i] = -9999.0;
	}
	
	while (lasreader->read_point())
	{
		point_r = lasreader->point;

		long I_a = int((point_r.get_y() - ymin) / lfDet);
		long J_a = int((point_r.get_x() - xmin) / lfDet);
		long I_b = int((point_r.get_y() - ymin - 0.5*lfDet) / lfDet); 
		long J_b = int((point_r.get_x() - xmin - 0.5*lfDet) / lfDet); 

		if (point_r.get_z() >= pHightIfo_a[I_a*nWidth + J_a])
		{
			pHightIfo_a[I_a*nWidth + J_a] = point_r.get_z();
		}
		if (point_r.get_z() >= pHightIfo_b[I_b*nWidth + J_b])
		{
			pHightIfo_b[I_b*nWidth + J_b] = point_r.get_z();
		}
	}
	lasreader->close();
	lasreader = NULL;

	lasreadopener.set_file_name(_szLasIn.c_str());

	if (!lasreadopener.active()) {
		std::cout << "ERROR: no input specified" << std::endl;
		return false;
	}

	lasreader = lasreadopener.open();
	if (!lasreader) {
		std::cerr << "ERROR: could not open LAS file: " << _szLasIn << std::endl;
		return false;
	}

	las_header_read = lasreader->header;
	las_header_read.user_data_after_header = nullptr;

	point_type = las_header_read.point_data_format;
	point_size = las_header_read.point_data_record_length;

	point_r.init(&las_header_read, point_type, point_size, &las_header_read);

	int surviving_number_of_point_records = 0;
	unsigned int surviving_number_of_points_by_return[] = { 0,0,0,0,0,0,0,0 };
	bool first_surviving_point = true;
	LASpoint surviving_point_min;
	LASpoint surviving_point_max;
	double surviving_gps_time_min = 0;
	double surviving_gps_time_max = 0;
	short surviving_rgb_min[3] = { 0,0,0 };
	short surviving_rgb_max[3] = { 0,0,0 };
	int scale_rgb = 0;

	int *nSurviveFlag = new int[npoints];
	for (int k = 0; k < npoints; k++) { nSurviveFlag[k] = 0; }

	double xyz_min[3];
	double xyz_max[3];

	int nCount = 0;

	while (lasreader->read_point())
	{
		nCount++;
		point_r =  lasreader->point;

		long I_a = int((point_r.get_y() - ymin) / lfDet);
		long J_a = int((point_r.get_x() - xmin) / lfDet); 
		long I_b = int((point_r.get_y() - ymin - 0.5*lfDet) / lfDet); 
		long J_b = int((point_r.get_x() - xmin - 0.5*lfDet) / lfDet); 

		if (((pHightIfo_a[I_a*nWidth + J_a] - point_r.get_z()) < hight_thresh &&
			(pHightIfo_a[I_a*nWidth + J_a] - point_r.get_z()) >= 0) &&
			((pHightIfo_b[I_b*nWidth + J_b] - point_r.get_z()) < hight_thresh &&
			(pHightIfo_b[I_b*nWidth + J_b] - point_r.get_z()) >= 0))
		{
			if (surviving_number_of_point_records == 0)
			{
				xyz_min[0] = point_r.get_x();
				xyz_min[1] = point_r.get_y();
				xyz_min[2] = point_r.get_z();
				xyz_max[0] = point_r.get_x();;
				xyz_max[1] = point_r.get_y();;
				xyz_max[2] = point_r.get_z();;
			}
			else
			{
				if (point_r.get_x() < xyz_min[0])
				{
					xyz_min[0] = point_r.get_x();
				}
				if (point_r.get_y() < xyz_min[1])
				{
					xyz_min[1] = point_r.get_y();
				}
				if (point_r.get_z() < xyz_min[2])
				{
					xyz_min[2] = point_r.get_z();
				}
				if (point_r.get_x() > xyz_max[0])
				{
					xyz_max[0] = point_r.get_x();
				}
				if (point_r.get_y() > xyz_max[1])
				{
					xyz_max[1] = point_r.get_y();
				}
				if (point_r.get_z() > xyz_max[2])
				{
					xyz_max[2] = point_r.get_z();
				}
			}
			nSurviveFlag[nCount - 1] = 1;
			surviving_number_of_point_records++;
			if (lasreader->point.return_number - 1 >= 0 && lasreader->point.return_number - 1 < 8)
			{
				surviving_number_of_points_by_return[lasreader->point.return_number - 1]++;
			}
			if (first_surviving_point)
			{
				surviving_point_min = lasreader->point;
				surviving_point_max = lasreader->point; 
				if (point_r.have_gps_time)
				{
					surviving_gps_time_min = point_r.get_gps_time();
					surviving_gps_time_max =point_r.get_gps_time();
				}
				if (point_r.have_rgb)
				{
					if (scale_rgb == 1)
					{
						point_r.rgb[0] = (point_r.rgb[0] / 256);
						point_r.rgb[1] = (point_r.rgb[1] / 256);
						point_r.rgb[2] = (point_r.rgb[2] / 256);
					}
					else if (scale_rgb == 2)
					{
						point_r.rgb[0] = (point_r.rgb[0] * 256);
						point_r.rgb[1] = (point_r.rgb[1] * 256);
						point_r.rgb[2] = (point_r.rgb[2] * 256);
					}
					surviving_rgb_min[0] =point_r.rgb[0];
					surviving_rgb_min[1] =point_r.rgb[1];
					surviving_rgb_min[2] =point_r.rgb[2];
					surviving_rgb_max[0] = point_r.rgb[0];
					surviving_rgb_max[1] = point_r.rgb[1];
					surviving_rgb_max[2] = point_r.rgb[2];
				}
				first_surviving_point = false;
			}
			else
			{
				if (point_r.get_X() < surviving_point_min.X)
				{
					surviving_point_min.X = point_r.get_X();
				}
				else if (point_r.get_X() > surviving_point_max.X)
				{
					surviving_point_max.X = point_r.get_X();
				}
				if (point_r.get_Y() < surviving_point_min.Y)
				{
					surviving_point_min.Y = point_r.get_Y();
				}
				else if (point_r.get_Y() > surviving_point_max.Y)
				{
					surviving_point_max.Y = point_r.get_Y();
				}
				if (point_r.get_Z() < surviving_point_min.Z)
				{
					surviving_point_min.Z = point_r.get_Z();
				}
				else if (point_r.get_Z() > surviving_point_max.Z) 
				{
					surviving_point_max.Z = point_r.get_Z();
				}

				//if (lasreader->point.intensity < surviving_point_min.intensity) //�ҵ�ǿ����Сֵ
				//{
				//	surviving_point_min.intensity = lasreader->point.intensity;
				//}
				//else if (lasreader->point.intensity > surviving_point_max.intensity) //�ҵ�ǿ�����ֵ
				//{
				//	surviving_point_max.intensity = lasreader->point.intensity;
				//}
				//if (lasreader->point.edge_of_flight_line < surviving_point_min.edge_of_flight_line)
				//{
				//	surviving_point_min.edge_of_flight_line = lasreader->point.edge_of_flight_line;
				//}
				//else if (lasreader->point.edge_of_flight_line > surviving_point_max.edge_of_flight_line)
				//{
				//	surviving_point_max.edge_of_flight_line = lasreader->point.edge_of_flight_line;
				//}
				//if (lasreader->point.scan_direction_flag < surviving_point_min.scan_direction_flag)
				//{
				//	surviving_point_min.scan_direction_flag = lasreader->point.scan_direction_flag;
				//}
				//else if (lasreader->point.scan_direction_flag > surviving_point_max.scan_direction_flag)
				//{
				//	surviving_point_max.scan_direction_flag = lasreader->point.scan_direction_flag;
				//}
				//if (lasreader->point.return_number < surviving_point_min.return_number)
				//{
				//	surviving_point_min.return_number = lasreader->point.return_number;
				//}
				//else if (lasreader->point.return_number > surviving_point_max.return_number)
				//{
				//	surviving_point_max.return_number = lasreader->point.return_number;
				//}
				//if (lasreader->point.classification < surviving_point_min.classification)
				//{
				//	surviving_point_min.classification = lasreader->point.classification;
				//}
				//else if (lasreader->point.classification > surviving_point_max.classification)
				//{
				//	surviving_point_max.classification = lasreader->point.classification;
				//}
				//if (lasreader->point.scan_angle_rank < surviving_point_min.scan_angle_rank)
				//{
				//	surviving_point_min.scan_angle_rank = lasreader->point.scan_angle_rank;
				//}
				//else if (lasreader->point.scan_angle_rank > surviving_point_max.scan_angle_rank)
				//{
				//	surviving_point_max.scan_angle_rank = lasreader->point.scan_angle_rank;
				//}
				//if (lasreader->point.user_data < surviving_point_min.user_data)
				//{
				//	surviving_point_min.user_data = lasreader->point.user_data;
				//}
				//else if (lasreader->point.user_data > surviving_point_max.user_data)
				//{
				//	surviving_point_max.user_data = lasreader->point.user_data;
				//}
				//if (lasreader->point.point_source_ID < surviving_point_min.point_source_ID)
				//{
				//	surviving_point_min.point_source_ID = lasreader->point.point_source_ID;
				//}
				//else if (lasreader->point.point_source_ID > surviving_point_max.point_source_ID)
				//{
				//	surviving_point_max.point_source_ID = lasreader->point.point_source_ID;
				//}
				//if (lasreader->point.point_source_ID < surviving_point_min.point_source_ID)
				//{
				//	surviving_point_min.point_source_ID = lasreader->point.point_source_ID;
				//}
				//else if (lasreader->point.point_source_ID > surviving_point_max.point_source_ID)
				//{
				//	surviving_point_max.point_source_ID = lasreader->point.point_source_ID;
				//}
				//if (point_r.have_gps_time)
				//{
				//	if (point_r.get_gps_time() < surviving_gps_time_min)
				//	{
				//		surviving_gps_time_min = point_r.get_gps_time();
				//	}
				//	else if (point_r.get_gps_time() > surviving_gps_time_max)
				//	{
				//		surviving_gps_time_max = point_r.get_gps_time();
				//	}
				//}
				//if (point_r.have_rgb)
				//{
				//	if (scale_rgb == 1)
				//	{
				//		point_r.rgb[0] = (point_r.rgb[0] / 256);
				//		point_r.rgb[1] = (point_r.rgb[1] / 256);
				//		point_r.rgb[2] = (point_r.rgb[2] / 256);
				//	}
				//	else if (scale_rgb == 2)
				//	{
				//		point_r.rgb[0] = (point_r.rgb[0] * 256);
				//		point_r.rgb[1] = (point_r.rgb[1] * 256);
				//		point_r.rgb[2] = (point_r.rgb[2] * 256);
				//	}
				//	surviving_rgb_min[0] = point_r.rgb[0];
				//	surviving_rgb_min[1] = point_r.rgb[1];
				//	surviving_rgb_min[2] = point_r.rgb[2];
				//	surviving_rgb_max[0] = point_r.rgb[0];
				//	surviving_rgb_max[1] = point_r.rgb[1];
				//	surviving_rgb_max[2] = point_r.rgb[2];
				//}
				//if (point_r.rgb[0] < surviving_rgb_min[0])
				//{
				//	surviving_rgb_min[0] = point_r.rgb[0];
				//}
				//else if (point_r.rgb[0] > surviving_rgb_max[0])
				//{
				//	surviving_rgb_max[0] = point_r.rgb[0];
				//}
				//if (point_r.rgb[1] < surviving_rgb_min[1])
				//{
				//	surviving_rgb_min[1] = point_r.rgb[1];
				//}
				//else if (point_r.rgb[1] > surviving_rgb_max[1])
				//{
				//	surviving_rgb_max[1] = point_r.rgb[1];
				//}
				//if (point_r.rgb[2] < surviving_rgb_min[2])
				//{
				//	surviving_rgb_min[2] = point_r.rgb[2];
				//}
				//else if (point_r.rgb[2] > surviving_rgb_max[2])
				//{
				//	surviving_rgb_max[2] = point_r.rgb[2];
				//}
			}
		}
	}

	lasreader->close();
	lasreader = NULL;
	delete[] pHightIfo_a; pHightIfo_a = NULL;
	delete[] pHightIfo_b; pHightIfo_b = NULL;

	lasreadopener.set_file_name(_szLasIn.c_str());

	if (!lasreadopener.active()) {
		std::cout << "ERROR: no input specified" << std::endl;
		return false;
	}

	lasreader = lasreadopener.open();
	if (!lasreader) {
		std::cerr << "ERROR: could not open LAS file: " << _szLasIn << std::endl;
		return false;
	}

	las_header_read = lasreader->header;
	las_header_read.user_data_after_header = nullptr;

	point_type = las_header_read.point_data_format;
	point_size = las_header_read.point_data_record_length;

	point_r.init(&las_header_read, point_type, point_size, &las_header_read);

	LASwriteOpener laswriteopener;
	LASheader            las_header_write;
	LASwriter*           laswriter;
	laswriteopener.set_file_name(_szLasOut.c_str());

	if (!laswriteopener.active()) {
		std::cerr << "error: could not write las file: " << _szLasOut << std::endl;
		exit(-1);
	}

	if (strcmp(&_szLasOut.back(), "z") == 0) {
		laswriteopener.set_format(LAS_TOOLS_FORMAT_LAZ);
	}
	else {
		laswriteopener.set_format(LAS_TOOLS_FORMAT_LAS);
	}
	//д�ļ�ͷ
	las_header_write.x_offset = las_header_read.x_offset;
	las_header_write.y_offset = las_header_read.y_offset;
	las_header_write.z_offset = las_header_read.z_offset;
	las_header_write.x_scale_factor = las_header_read.x_scale_factor;
	las_header_write.y_scale_factor = las_header_read.y_scale_factor;
	las_header_write.z_scale_factor = las_header_read.z_scale_factor;

	for (int i = 0; i < 4; i++)
		las_header_write.file_signature[i] = las_header_read.file_signature[i];

	las_header_write.file_source_ID = las_header_read.file_source_ID;
	las_header_write.global_encoding = las_header_read.global_encoding;
	las_header_write.project_ID_GUID_data_1 = las_header_read.project_ID_GUID_data_1;
	las_header_write.project_ID_GUID_data_2 = las_header_read.project_ID_GUID_data_2;
	las_header_write.project_ID_GUID_data_3 = las_header_read.project_ID_GUID_data_3;
	for (int i = 0; i < 8; i++)
		las_header_write.project_ID_GUID_data_4[i] = las_header_read.project_ID_GUID_data_4[i];

	las_header_write.version_major = las_header_read.version_major; // 1
	las_header_write.version_minor = las_header_read.version_minor; // 2
	for (int i = 0; i < 32; i++)
	{
		las_header_write.system_identifier[i] = las_header_read.system_identifier[i];
		las_header_write.generating_software[i] = las_header_read.generating_software[i];
	}
	las_header_write.file_creation_day = las_header_read.file_creation_day;
	las_header_write.file_creation_year = las_header_read.file_creation_year;

	las_header_write.point_data_format = las_header_read.point_data_format;
	las_header_write.point_data_record_length = las_header_read.point_data_record_length;

	laswriter = laswriteopener.open(&las_header_write);
	if (!laswriter) {
		return false;
	}

	LASpoint laspoint_w;
	if (!laspoint_w.init(&las_header_write, las_header_write.point_data_format, las_header_write.point_data_record_length, 0)) {
		return EXIT_FAILURE;
	}

	nCount = 0;
	while (lasreader->read_point())
	{
		nCount++;
		if (0 == nSurviveFlag[nCount - 1])
			continue;

		// write the first point
		laswriter->write_point(&lasreader->point);
		laswriter->update_inventory(&lasreader->point);
	}
	delete[] nSurviveFlag; nSurviveFlag = NULL;

	las_header_write.number_of_point_records = surviving_number_of_point_records;
	for (int i = 0; i < 5; i++)
		las_header_write.number_of_points_by_return[i] = surviving_number_of_points_by_return[i];

	las_header_write.min_x = surviving_point_min.X*las_header_write.x_scale_factor + las_header_write.x_offset;
	las_header_write.max_x = surviving_point_max.X*las_header_write.x_scale_factor + las_header_write.x_offset;
	las_header_write.min_y = surviving_point_min.Y*las_header_write.y_scale_factor + las_header_write.y_offset;
	las_header_write.max_y = surviving_point_max.Y*las_header_write.y_scale_factor + las_header_write.y_offset;
	las_header_write.min_z = surviving_point_min.Z*las_header_write.z_scale_factor + las_header_write.z_offset;
	las_header_write.max_z = surviving_point_max.Z*las_header_write.z_scale_factor + las_header_write.z_offset;
	laswriter->update_header(&las_header_write, true);

	lasreader->close();
	lasreader = NULL;

	laswriter->close();
	laswriter = NULL;

	return true;
}


bool LasFilter::run(std::string szLasIn,
	std::string szLasOut,
	double lfLdrPtInterv,
	bool GridFilterFlag)
{
	std::string drive, dir, fname, ext;
	splitpath(szLasIn, drive, dir, fname, ext);

	std::string szLasOutTemp = dir + fname + "_dp" + ext;
	if (!deltGross(szLasIn, szLasOutTemp, lfLdrPtInterv))
	{
		std::cout << "Unable to fliter noise points!";
		return false;
	}

	if (GridFilterFlag)
	{
		if (!gridFilter(szLasOutTemp, szLasOut, 1.2*lfLdrPtInterv, 0.25))
		{
			std::cout << "Unable to remove wall points!";
			return false;
		}
		remove(szLasOutTemp.c_str());
	}

	return true;
}