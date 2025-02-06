#include "multiAgentMapping/distributed_mapping/lidarIrisDescriptor.hpp"

lidar_iris_descriptor::lidar_iris_descriptor (
        int rows = 80,
        int columns = 360,
        int n_scan = 128,
        double distance_threshold = 0.32,
        int exclude_recent_frame_num = 30,
        int match_mode = 2,
        int candidates_num = 10,
        int nscale = 4,
        int min_wave_length = 18,
        float mult = 1.6,
        float sigma_on_f = 0.75,
        int robot_num = 1,
        int id = 0):
    rows_(rows), // 80 in the original paper
    columns_(columns), // 360 in the original paper
    n_scan_(n_scan), // lidar sensor lines
    distance_threshold_(distance_threshold), // 0.10-0.45 is ok.
    // 0.10-0.15 is very fine (rare false-alarms);
    // 0.15-0.35 is good but ICP fitness score check should be required for robustness;
    // 0.35-0.45 is choice for using with robust outlier rejection.
    exclude_recent_frame_num_(exclude_recent_frame_num), // simply just keyframe gap
    match_mode_(match_mode), // 0 for detecting same direction
    // 1 for detecting opposite direction
    // 2 for detecting both same and opposite direction
    candidates_num_(candidates_num), // 6-10 is enough
    nscale_(nscale), // 4 in the original paper
    min_wave_length_(min_wave_length), // 18 in the original paper
    mult_(mult), // 1.6 in the original paper
    sigma_on_f_(sigma_on_f), // 0.75 in the original paper
    robot_num_(robot_num), // number of robot in robotic swarm
    id_(id) // this robot id
{
    for (int i = 0; i < robot_num_; i++) {
        unordered_map < int, lidar_iris_descriptor::featureDesc > base_feature_desc;
        unordered_map < int, Eigen::MatrixXf > base_matrix;
        unordered_map < int, int > base_int;
        iris_features.push_back(base_feature_desc);
        iris_rowkeys.push_back(base_matrix);
        indexes_maps.push_back(base_int);
    }
}

lidar_iris_descriptor::~lidar_iris_descriptor(){}

std::pair<Eigen::VectorXf, cv::Mat1b> lidar_iris_descriptor::getIris(
    const pcl::PointCloud<pcl::PointXYZI> &cloud
){
    cv::Mat1b iris_image = cv::Mat1b::zeros(rows_, columns_);
    Eigen::MatrixXf iris_row_key_matrix = Eigen::MatrixXf::Zero(rows_, columns_);
    // VLS 128 lidar logic
    if(n_scan_ == 128){ 
        for(auto p : cloud.points){
            // compute XY-plane distance
            float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);

            // vertical angle remapping: [-25 to +15] -> [0, 40]
            float arc = (atan2(p.data[2], dis) * 180.0f / M_PI) + 25;

            // horizontal angle remapping [-180, +180] -> [0, 360]
            float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;

            // discrtize into bins
            int Q_dis = std::min(std::max((int)floor(dis), 0), (rows_ - 1));
            int Q_arc = std::min(std::max((int)floor(arc / 5.0f), 0), 7);  // 8 bins, 5° each
            int Q_yaw = std::min(std::max((int)floor(yaw + 0.5), 0), (columns_ - 1));

            // update iris image with bit-wise encoding
            iris_image.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc);

            // Update the row key matrix to store the maximum height (z-value)
            if(iris_row_key_matrix(Q_dis, Q_yaw) < p.data[2]){
                iris_row_key_matrix(Q_dis, Q_yaw) = p.data[2];
            }
        }
    } else {
        std::cout << "[LiDAR Iris Descriptor] Error : Only n_scan = 128 supported" << std::endl;
    }

    // extract rowkey
    Eigen::VectorXf rowkey = Eigen::VectorXf::Zero(rows_);
    for (int i = 0; i < iris_row_key_matrix.rows(); i++){
        Eigen::VectorXf curr_row = iris_row_key_matrix.row(i);
        rowkey(i) = curr_row.mean();
    }

    return std::make_pair(rowkey, iris_image);
}

inline cv::Mat lidar_iris_descriptor::circularRowShift(
    const cv::Mat &src,
    int shift_m_rows
){
    if(shift_m_rows == 0){
        return src.clone();
    }
    shift_m_rows %= src.rows;
    int m = shift_m_rows > 0 ? shift_m_rows : src.rows + shift_m_rows;
    cv::Mat dst(src.size(), src.type());
    src(cv::Range(src.rows - m, src.rows), cv::Range::all()).copyTo(dst(cv::Range(0, m), cv::Range::all()));
	src(cv::Range(0, src.rows - m), cv::Range::all()).copyTo(dst(cv::Range(m, src.rows), cv::Range::all()));
	return dst;
}

inline cv::Mat lidar_iris_descriptor::circularColShift(
    const cv::Mat &src,
    int shift_n_cols
){
    if(shift_n_cols == 0){
        return src.clone();
    }
    shift_n_cols %= src.cols;
    int n = shift_n_cols > 0 ? shift_n_cols : src.cols + shift_n_cols;
    cv::Mat dst(src.size(), src.type());
	src(cv::Range::all(), cv::Range(src.cols - n, src.cols)).copyTo(dst(cv::Range::all(), cv::Range(0, n)));
	src(cv::Range::all(), cv::Range(0, src.cols - n)).copyTo(dst(cv::Range::all(), cv::Range(n, src.cols)));
	return dst;
}

cv::Mat lidar_iris_descriptor::circularShift(
	const cv::Mat &src,
	int shift_m_rows,
	int shift_n_cols)
{
	return circularColShift(circularRowShift(src, shift_m_rows), shift_n_cols);
}

std::vector<cv::Mat2f> lidar_iris_descriptor::logGaborFilter(
    const cv::Mat2f &src,
    unsigned int nscale,
    int min_wave_length,
    double mult,
    double_sigma_on_f
){
    // get dim for input matrix
    int rows = src.rows;
    int cols = src.cols;

    // initialize the filter sum matrix (used later for frequency shifts)
    cv__Mat2f filtersum = cv::Mat2f::zeros(1, cols);
    std::vector<cv::Mat2f> EO(nscale); // vector to store the filtered output at each scale
	int ndata = cols; // number of data points -> ensure its even
	if(ndata % 2 == 1) // if odd, reduce by 1
	{
		ndata--;
	}

    // matrix for storing log gabor filter at each scale
    cv::Mat1f logGabor = cv::Mat1f::zeros(1, ndata);

    // matrix for storing the result after filter applied
    cv::Mat2f result = cv::Mat2f::zeros(rows, ndata);

    // create a frequency radius matrix [0, 1/ndata, 2/ndata, ....]
    cv::Mat1f radius = cv::Mat1f::zeros(1, ndata / 2 + 1);
    radius.at<float>(0, 0) = 1; // avoid division by zero
    for(int i = 1; i < ndata / 2 + 1; i++)
	{
		radius.at<float>(0, i) = i / (float)ndata;
	}
    // initialise wavelength for the first scale
	double wavelength = min_wave_length;

    // iterate over each scale to create and apply the log-gabor filter
    for(int s = 0; s < nscale; s++)
	{
        // Compute the center frequency of the current scale
		double fo = 1.0 / wavelength;   // Center frequency (spatial domain)
		double rfo = fo / 0.5;          // Ratio of center frequency to Nyquist frequency
		
        // Calculate the Log-Gabor filter in the frequency domain
		cv::Mat1f temp; //(radius.size());
		cv::Mat1f radius_tmp = radius / fo;
		cv::log(radius_tmp, temp);                  // log(radius / fo)
		cv::pow(temp, 2, temp);                     // (log(radius / fo))^2
		cv::Mat1f mid_result = (-temp) / (2 * log(sigma_on_f) * log(sigma_on_f));
		cv::exp(mid_result, temp);                  // exp(-log^2(radius / fo) / (2 * sigma_on_f^2))
		temp.copyTo(logGabor.colRange(0, ndata / 2 + 1));
		
        // Set the DC component (logGabor[0]) to 0 to suppress low frequencies
		logGabor.at<float>(0, 0) = 0;

        // Create the complex filter by merging the Log-Gabor filter with a zero imaginary part
		cv::Mat2f filter;
		cv::Mat1f filterArr[2] = {logGabor, cv::Mat1f::zeros(logGabor.size())};
		cv::merge(filterArr, 2, filter);
        // Accumulate the filter for frequency shifting (used later)
		filtersum = filtersum + filter;
        // Apply the filter to each row of the input matrix (2D DFT)
		for(int r = 0; r < rows; r++)
		{
            // Convert the real input row to a complex row with a zero imaginary part
			cv::Mat2f src2f;
			cv::Mat1f srcArr[2] = {src.row(r).clone(), cv::Mat1f::zeros(1, src.cols)};
			cv::merge(srcArr, 2, src2f);

            // Perform forward Discrete Fourier Transform (DFT)
			cv::dft(src2f, src2f);
			cv::mulSpectrums(src2f, filter, src2f, 0); // Multiply the spectrum with the Log-Gabor filter
			cv::idft(src2f, src2f); // Perform inverse DFT to get the filtered result
			src2f.copyTo(result.row(r)); // Copy the filtered result to the corresponding row in the result matrix
		}
        // Store the filtered result for the current scale
		EO[s] = result.clone();
        // Update the wavelength for the next scale
		wavelength *= mult;
	}
    // Circularly shift the filter sum to center the frequency response
	filtersum = circShift(filtersum, 0, cols / 2);
    // Return the filtered responses at all scales
	return EO;
}

void lidar_iris_descriptor::logFeatureEncode(
    const cv::Mat1b &src,           // Input binary iris image
    unsigned int nscale,            // Number of scales for the Log-Gabor filter
    int min_wave_length,            // Minimum wavelength of the Log-Gabor filter
    double mult,                    // Scaling factor between successive filters
    double sigma_on_f,              // Standard deviation of the filter’s Gaussian envelope
    cv::Mat1b &T,                   // Output binary matrix encoding phase thresholding
    cv::Mat1b &M                    // Output binary matrix encoding low-magnitude filtering
)
{
    // Step 1: Convert the input binary matrix to a floating-point format
    cv::Mat1f src_float;
    src.convertTo(src_float, CV_32FC1);  // Convert CV_8U to CV_32F for DFT-based processing

    // Step 2: Apply the Log-Gabor filter to the input matrix at multiple scales
    auto list = logGaborFilter(src_float, nscale, min_wave_length, mult, sigma_on_f);

    // Step 3: Initialize binary matrices for phase thresholding (Tlist) and magnitude filtering (Mlist)
    std::vector<cv::Mat1b> Tlist(nscale * 2);  // Stores phase threshold results for real and imaginary parts
    std::vector<cv::Mat1b> Mlist(nscale * 2);  // Stores low-magnitude filtering results for both parts

    // Step 4: Process the filtered responses for each scale
    for (int i = 0; i < list.size(); i++)
    {
        // Split the filtered response into real and imaginary components
        cv::Mat1f arr[2];
        cv::split(list[i], arr);  // arr[0]: real part, arr[1]: imaginary part

        // Step 5: Perform phase thresholding
        // Threshold the real and imaginary parts: T = 1 if value > 0, otherwise 0
        Tlist[i] = arr[0] > 0;              // Threshold the real part
        Tlist[i + nscale] = arr[1] > 0;     // Threshold the imaginary part

        // Step 6: Compute magnitude and perform low-magnitude filtering
        cv::Mat1f m;
        cv::magnitude(arr[0], arr[1], m);   // Compute the magnitude of the complex number
        Mlist[i] = m < 0.0001;              // Set M = 1 if magnitude is near 0 (low magnitude)
        Mlist[i + nscale] = m < 0.0001;     // Apply the same for the imaginary part
    }

    // Step 7: Vertically concatenate all binary matrices for phase and magnitude results
    cv::vconcat(Tlist, T);  // Concatenate phase thresholding results into a single matrix
    cv::vconcat(Mlist, M);  // Concatenate low-magnitude filtering results into a single matrix
}

// This function encodes the given binary image into a feature descriptor containing phase and magnitude information
lidar_iris_descriptor::featureDesc lidar_iris_descriptor::getFeature(const cv::Mat1b &src)
{
    lidar_iris_descriptor::featureDesc desc;  // Initialize a feature descriptor to hold the results
    desc.img = src;  // Store the input binary image

    // Encode the image using Log-Gabor filtering to compute phase (T) and magnitude (M) descriptors
    logFeatureEncode(src, nscale_, min_wave_length_, mult_, sigma_on_f_, desc.T, desc.M);

    // Return the final descriptor containing the binary image, phase thresholding matrix (T), and magnitude matrix (M)
    return desc;
}

// This version of getFeature extracts an additional feature vector based on row averages
lidar_iris_descriptor::featureDesc lidar_iris_descriptor::getFeature(
    const cv::Mat1b &src,
    std::vector<float> &vec)
{
    cv::Mat1f temp;  // Temporary floating-point matrix to store intermediate results

    // Step 1: Convert the input binary matrix to floating-point format for processing
    src.convertTo(temp, CV_32FC1);

    // Step 2: Compute row-wise averages where binary values are non-zero
    // Divide by 255 to normalize and convert binary values (0 or 255) to 0 or 1
    cv::reduce((temp != 0) / 255, temp, 1, cv::REDUCE_AVG);

    // Step 3: Store the row-wise averages in the feature vector
    // Ensure the vector is continuous in memory to avoid allocation issues
    vec = temp.isContinuous() ? temp : temp.clone();

    // Step 4: Call the main getFeature function to generate phase and magnitude descriptors
    return getFeature(src);
}

void lidar_iris_descriptor::recomb(
    cv::Mat &src,  // Input matrix to recombine
    cv::Mat &dst   // Output matrix after recombination
)
{
    // Step 1: Determine the center coordinates of the image
    int cx = src.cols >> 1;  // Half the width (right shift by 1 divides by 2)
    int cy = src.rows >> 1;  // Half the height

    // Step 2: Create a temporary matrix to store the recombined result
    cv::Mat tmp;
    tmp.create(src.size(), src.type());

    // Step 3: Swap the four quadrants of the image
    // Move quadrant 1 (top-left) to quadrant 4 (bottom-right)
    src(cv::Rect(0, 0, cx, cy)).copyTo(tmp(cv::Rect(cx, cy, cx, cy)));

    // Move quadrant 4 (bottom-right) to quadrant 1 (top-left)
    src(cv::Rect(cx, cy, cx, cy)).copyTo(tmp(cv::Rect(0, 0, cx, cy)));

    // Move quadrant 2 (top-right) to quadrant 3 (bottom-left)
    src(cv::Rect(cx, 0, cx, cy)).copyTo(tmp(cv::Rect(0, cy, cx, cy)));

    // Move quadrant 3 (bottom-left) to quadrant 2 (top-right)
    src(cv::Rect(0, cy, cx, cy)).copyTo(tmp(cv::Rect(cx, 0, cx, cy)));

    // Step 4: Set the output matrix to the recombined result
    dst = tmp;
}

void lidar_iris_descriptor::forwardFFT(
    cv::Mat &src,          // Input 2D image (grayscale or single-channel)
    cv::Mat *f_img,        // Output array to store real and imaginary parts of the FFT
    bool do_recomb          // Whether to recombine (shift) the frequency components to the center
)
{
    // Step 1: Get optimal DFT sizes for more efficient computation
    int M = cv::getOptimalDFTSize(src.rows);  // Optimal number of rows
    int N = cv::getOptimalDFTSize(src.cols);  // Optimal number of columns

    // Step 2: Pad the image to the optimal size using zero-padding
    cv::Mat padded;
    copyMakeBorder(src, padded, 0, M - src.rows, 0, N - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    // Step 3: Prepare planes for the real and imaginary components
    cv::Mat planes[] = {
        cv::Mat_<float>(padded),                  // Real part (initialized with the input image)
        cv::Mat::zeros(padded.size(), CV_32F)     // Imaginary part (initialized to zeros)
    };

    // Step 4: Merge the real and imaginary parts into a complex image
    cv::Mat complex_img;
    merge(planes, 2, complex_img);  // Merge into a 2-channel complex image

    // Step 5: Perform the 2D Discrete Fourier Transform (DFT)
    dft(complex_img, complex_img);

    // Step 6: Split the complex image back into real and imaginary parts
    split(complex_img, planes);

    // Step 7: Ensure the dimensions of the output planes are even
    // This is done by trimming any odd rows or columns for consistent processing
    planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
    planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));

    // Step 8: Optionally recombine (shift) the quadrants of the frequency components
    // This moves the zero-frequency component to the center of the image
    if (do_recomb)
    {
        recomb(planes[0], planes[0]);  // Shift the real part
        recomb(planes[1], planes[1]);  // Shift the imaginary part
    }

    // Step 9: Normalize the frequency components by dividing by the total number of elements
    planes[0] /= float(M * N);  // Normalize the real part
    planes[1] /= float(M * N);  // Normalize the imaginary part

    // Step 10: Store the normalized real and imaginary parts in the output array
    f_img[0] = planes[0].clone();  // Copy the real part to the output
    f_img[1] = planes[1].clone();  // Copy the imaginary part to the output
}

void lidar_iris_descriptor::highpass(cv::Size sz, cv::Mat &dst)
{
    // Step 1: Create 1D arrays to represent cosine components along the y and x axes
    cv::Mat a = cv::Mat(sz.height, 1, CV_32FC1);  // Vertical cosine component
    cv::Mat b = cv::Mat(1, sz.width, CV_32FC1);   // Horizontal cosine component

    // Step 2: Calculate the step sizes for moving along the cosine curve
    float step_y = CV_PI / sz.height;  // Step size for vertical cosine function
    float val = -CV_PI * 0.5;          // Start value for vertical cosine (begin at -π/2)

    // Step 3: Compute the vertical cosine values (from -π/2 to π/2)
    for (int i = 0; i < sz.height; ++i)
    {
        a.at<float>(i) = cos(val);  // Compute cosine value for each row
        val += step_y;              // Increment to the next position along the y-axis
    }

    // Step 4: Reset the starting value and calculate the step size for the horizontal cosine
    val = -CV_PI * 0.5;            // Start at -π/2 again for the horizontal axis
    float step_x = CV_PI / sz.width;  // Step size for horizontal cosine function

    // Step 5: Compute the horizontal cosine values (from -π/2 to π/2)
    for (int i = 0; i < sz.width; ++i)
    {
        b.at<float>(i) = cos(val);  // Compute cosine value for each column
        val += step_x;              // Increment to the next position along the x-axis
    }

    // Step 6: Compute the outer product of the vertical and horizontal cosine values
    // This creates a 2D cosine matrix where each value represents a combination of the x and y components
    cv::Mat tmp = a * b;

    // Step 7: Apply the high-pass filter equation: (1.0 - tmp) * (2.0 - tmp)
    // This enhances higher-frequency components while attenuating lower-frequency (DC) components
    dst = (1.0 - tmp).mul(2.0 - tmp);
}

float lidar_iris_descriptor::logpolar(
    cv::Mat &src,  // Input image (typically a 2D matrix)
    cv::Mat &dst   // Output image in log-polar coordinates
)
{
    // Step 1: Determine the number of radii and angles based on the input image dimensions
    float radii = src.cols;    // Number of radial bins
    float angles = src.rows;   // Number of angular bins

    // Step 2: Find the center point of the image
    cv::Point2f center(src.cols / 2, src.rows / 2);

    // Step 3: Calculate the maximum distance from the center to the farthest corner
    float d = cv::norm(cv::Vec2f(src.cols - center.x, src.rows - center.y));

    // Step 4: Compute the base of the logarithm for the log-polar transformation
    // The base is derived to map the entire distance range (0 to d) to the available number of radii
    float log_base = std::pow(10.0, log10(d) / radii);

    // Step 5: Determine the angular step size (d_theta) for each angle
    float d_theta = CV_PI / (float)angles;  // Half-circle (π) divided by the number of angles
    float theta = CV_PI / 2.0;  // Start at π/2 (90 degrees) for proper orientation

    // Step 6: Initialize mapping matrices to store the (x, y) coordinates corresponding to each (radius, angle)
    cv::Mat map_x(src.size(), CV_32FC1);
    cv::Mat map_y(src.size(), CV_32FC1);

    // Step 7: Generate the mapping matrices by iterating over each angle and radius
    for (int i = 0; i < angles; ++i)  // For each angle (row in the output image)
    {
        for (int j = 0; j < radii; ++j)  // For each radius (column in the output image)
        {
            // Compute the radius using the log-polar mapping
            radius = std::pow(log_base, float(j));

            // Compute the Cartesian coordinates (x, y) from the log-polar coordinates
            float x = radius * sin(theta) + center.x;
            float y = radius * cos(theta) + center.y;

            // Store the computed coordinates in the mapping matrices
            map_x.at<float>(i, j) = x;
            map_y.at<float>(i, j) = y;
        }

        // Increment the angle for the next row
        theta += d_theta;
    }

    // Step 8: Perform the remapping from Cartesian coordinates to log-polar coordinates
    cv::remap(src, dst, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // Step 9: Return the logarithmic base used in the transformation
    return log_base;
}

cv::RotatedRect lidar_iris_descriptor::logPolarFFTTemplateMatch(
    cv::Mat &im0,  // First input image
    cv::Mat &im1   // Second input image to match
    /*, double canny_threshold1, double canny_threshold2*/) // Uncomment for edge-based processing if needed
{
    // Step 1: Ensure input images have compatible types and dimensions
    CV_Assert((im0.type() == CV_8UC1) || (im0.type() == CV_8UC3) ||
              (im0.type() == CV_32FC1) || (im0.type() == CV_32FC3) ||
              (im0.type() == CV_64FC1) || (im0.type() == CV_64FC3));
    CV_Assert(im0.rows == im1.rows && im0.cols == im1.cols);
    CV_Assert(im0.channels() == 1 || im0.channels() == 3 || im0.channels() == 4);
    CV_Assert(im1.channels() == 1 || im1.channels() == 3 || im1.channels() == 4);

    // Step 2: Convert to grayscale if the input images have multiple channels
    if (im0.channels() == 3) cv::cvtColor(im0, im0, cv::COLOR_BGR2GRAY);
    if (im0.channels() == 4) cv::cvtColor(im0, im0, cv::COLOR_BGRA2GRAY);
    if (im1.channels() == 3) cv::cvtColor(im1, im1, cv::COLOR_BGR2GRAY);
    if (im1.channels() == 4) cv::cvtColor(im1, im1, cv::COLOR_BGRA2GRAY);

    // Step 3: Convert to CV_8UC1 if needed, then scale to CV_32FC1 for FFT processing
    if (im0.type() == CV_32FC1 || im0.type() == CV_64FC1) im0.convertTo(im0, CV_8UC1, 255.0);
    if (im1.type() == CV_32FC1 || im1.type() == CV_64FC1) im1.convertTo(im1, CV_8UC1, 255.0);
    im0.convertTo(im0, CV_32FC1, 1.0 / 255.0);
    im1.convertTo(im1, CV_32FC1, 1.0 / 255.0);

    // Step 4: Perform the forward FFT on both images
    cv::Mat F0[2], F1[2];  // Store real and imaginary parts of the FFT
    cv::Mat f0, f1;        // Magnitude images of the FFT
    forwardFFT(im0, F0);
    forwardFFT(im1, F1);
    cv::magnitude(F0[0], F0[1], f0);
    cv::magnitude(F1[0], F1[1], f1);

    // Step 5: Apply a high-pass filter to remove low-frequency components
    cv::Mat h;
    highpass(f0.size(), h);
    f0 = f0.mul(h);
    f1 = f1.mul(h);

    // Step 6: Convert to log-polar coordinates for scale and rotation invariance
    float log_base;
    cv::Mat f0lp, f1lp;
    log_base = logpolar(f0, f0lp);
    log_base = logpolar(f1, f1lp);

    // Step 7: Use phase correlation to estimate rotation and scaling
    cv::Point2d rotation_and_scale = cv::phaseCorrelate(f1lp, f0lp);
    float angle = 180.0 * rotation_and_scale.y / f0lp.rows;  // Convert to degrees
    float scale = std::pow(log_base, rotation_and_scale.x);  // Estimate the scaling factor

    // Step 8: Handle large scale changes (> 1.8x) by re-evaluating with inverted scale
    if (scale > 1.8)
    {
        rotation_and_scale = cv::phaseCorrelate(f1lp, f0lp);
        angle = -180.0 * rotation_and_scale.y / f0lp.rows;
        scale = 1.0 / std::pow(log_base, rotation_and_scale.x);
        if (scale > 1.8)
        {
            std::cout << "Images are not compatible. Scale change > 1.8" << std::endl;
            return cv::RotatedRect();  // Return an empty rectangle if scale change is too large
        }
    }

    // Step 9: Normalize the angle to be within the range [-90°, 90°]
    if (angle < -90.0)
        angle += 180.0;
    else if (angle > 90.0)
        angle -= 180.0;

    // Step 10: Apply the inverse rotation and scaling to align the images
    cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point(im1.cols / 2, im1.rows / 2), angle, 1.0 / scale);
    cv::Mat im1_rs;
    cv::warpAffine(im1, im1_rs, rot_mat, im1.size());  // Apply the transformation

    // Step 11: Find the translation using phase correlation between the aligned images
    cv::Point2d tr = cv::phaseCorrelate(im1_rs, im0);

    // Step 12: Compute the rotated rectangle that describes the transformation
    cv::RotatedRect rr;
    rr.center = tr + cv::Point2d(im0.cols / 2, im0.rows / 2);  // Center of the transformation
    rr.angle = -angle;  // Rotation angle
    rr.size.width = im1.cols / scale;  // Width after scaling
    rr.size.height = im1.rows / scale;  // Height after scaling

    return rr;  // Return the computed rotated rectangle
}

cv::RotatedRect lidar_iris_descriptor::fftMatch(
	const cv::Mat& im0,
	const cv::Mat& im1)
{
	cv::Mat im0_tmp = im0.clone();
	cv::Mat im1_tmp = im1.clone();
	return logPolarFFTTemplateMatch(im0_tmp, im1_tmp);
}

void lidar_iris_descriptor::getHammingDistance(
	const cv::Mat1b &T1,
	const cv::Mat1b &M1,
	const cv::Mat1b &T2,
	const cv::Mat1b &M2,
	int scale,
	float &dis,
	int &bias)
{
	dis = NAN;
	bias = -1;
	// #pragma omp parallel for num_threads(8)
	for(int shift = scale - 2; shift <= scale + 2; shift++)
	{
		cv::Mat1b T1s = circShift(T1, 0, shift);
		cv::Mat1b M1s = circShift(M1, 0, shift);
		cv::Mat1b mask = M1s | M2;
		cv::Mat1b mask_tmp = mask / 255;
		int MaskBitsNum = cv::sum(mask_tmp)[0];
		int totalBits = T1s.rows * T1s.cols - MaskBitsNum;
		cv::Mat1b C = T1s ^ T2;
		C = C & ~mask;
		cv::Mat1b c_tmp = C / 255;
		int bitsDiff = cv::sum(c_tmp)[0];
		if(totalBits == 0)
		{
			dis = NAN;
		}
		else
		{
			float currentDis = bitsDiff / (float)totalBits;
			if(currentDis < dis || isnan(dis))
			{
				dis = currentDis;
				bias = shift;
			}
		}
	}
	return;
}

void lidar_iris_descriptor::getHammingDistance(
    const cv::Mat1b &T1,  // Binary phase descriptor of the first feature
    const cv::Mat1b &M1,  // Binary mask of the first feature (low-magnitude bits)
    const cv::Mat1b &T2,  // Binary phase descriptor of the second feature
    const cv::Mat1b &M2,  // Binary mask of the second feature (low-magnitude bits)
    int scale,            // Initial scale for shifting
    float &dis,           // Output: minimum normalized Hamming distance
    int &bias             // Output: best shift value corresponding to minimum distance
)
{
    // Step 1: Initialize outputs
    dis = NAN;  // Initialize distance to NaN to indicate no distance found yet
    bias = -1;  // Initialize bias to -1 to indicate no shift found yet

    // Step 2: Try different shifts of the first binary descriptor within the range [scale-2, scale+2]
    for (int shift = scale - 2; shift <= scale + 2; shift++)
    {
        // Step 3: Apply circular row shift to T1 and M1 by `shift` columns
        cv::Mat1b T1s = circShift(T1, 0, shift);  // Circular shift for the phase descriptor
        cv::Mat1b M1s = circShift(M1, 0, shift);  // Circular shift for the mask

        // Step 4: Compute the combined mask where bits are considered invalid
        cv::Mat1b mask = M1s | M2;  // Combine the masks using bitwise OR to identify invalid bits
        cv::Mat1b mask_tmp = mask / 255;  // Normalize the mask to binary (0 or 1)
        int MaskBitsNum = cv::sum(mask_tmp)[0];  // Count the number of invalid bits (masked bits)

        // Step 5: Calculate the total number of valid bits (those not masked)
        int totalBits = T1s.rows * T1s.cols - MaskBitsNum;

        // Step 6: Compute the bitwise XOR between the shifted phase descriptor and T2
        cv::Mat1b C = T1s ^ T2;  // XOR to identify bit differences between T1s and T2

        // Step 7: Mask out invalid bits from the XOR result
        C = C & ~mask;  // Only consider valid bits (bits not masked)
        cv::Mat1b c_tmp = C / 255;  // Normalize the result to binary (0 or 1)
        int bitsDiff = cv::sum(c_tmp)[0];  // Count the number of differing bits

        // Step 8: Calculate the normalized Hamming distance
        if (totalBits == 0)  // Avoid division by zero if no valid bits are left
        {
            dis = NAN;
        }
        else
        {
            float currentDis = bitsDiff / (float)totalBits;  // Normalize the differing bits by valid bits

            // Step 9: Update the minimum distance and corresponding bias if a lower distance is found
            if (currentDis < dis || std::isnan(dis))  // Update if current distance is lower or dis is NaN
            {
                dis = currentDis;
                bias = shift;
            }
        }
    }

    return;
}

float lidar_iris_descriptor::compare(
    const lidar_iris_descriptor::featureDesc &img1,  // First feature descriptor
    const lidar_iris_descriptor::featureDesc &img2,  // Second feature descriptor
    int *bias                                       // Output: optimal shift value (in degrees)
)
{
    // If match_mode_ == 2, compare both the original and the shifted (180 degrees) versions
    if (match_mode_ == 2)
    {
        float dis1;  // Minimum Hamming distance for the original orientation
        int bias1;   // Corresponding shift value for the original orientation
        float dis2 = 0;  // Minimum Hamming distance for the 180-degree-shifted version
        int bias2 = 0;   // Corresponding shift value for the 180-degree-shifted version

        // #pragma omp parallel for num_threads(8)   // Optional parallelization for performance
        for (int i = 0; i < 2; i++)
        {
            if (i == 0)
            {
                // Step 1a: Perform FFT-based template matching for the original orientation
                auto firstRect = fftMatch(img2.img, img1.img);
                int firstShift = firstRect.center.x - img1.img.cols / 2;

                // Step 2a: Compute the Hamming distance between the original features
                getHammingDistance(img1.T, img1.M, img2.T, img2.M, firstShift, dis1, bias1);
            }
            else
            {
                // Step 1b: Perform FFT-based template matching for the 180-degree-shifted orientation
                auto T2x = circShift(img2.T, 0, 180);
                auto M2x = circShift(img2.M, 0, 180);
                auto img2x = circShift(img2.img, 0, 180);

                auto secondRect = fftMatch(img2x, img1.img);
                int secondShift = secondRect.center.x - img1.img.cols / 2;

                // Step 2b: Compute the Hamming distance between the shifted features
                getHammingDistance(img1.T, img1.M, T2x, M2x, secondShift, dis2, bias2);
            }
        }

        // Step 3: Select the orientation with the minimum Hamming distance
        if (dis1 < dis2)
        {
            if (bias)
                *bias = bias1;  // Set the bias to the shift value for the original orientation
            return dis1;        // Return the minimum Hamming distance for the original orientation
        }
        else
        {
            if (bias)
                *bias = (bias2 + 180) % 360;  // Adjust the bias for the 180-degree shift
            return dis2;  // Return the minimum Hamming distance for the shifted orientation
        }
    }

    // If match_mode_ == 1, only compare the 180-degree-shifted version
    if (match_mode_ == 1)
    {
        // Step 1: Shift the second feature by 180 degrees
        auto T2x = circShift(img2.T, 0, 180);
        auto M2x = circShift(img2.M, 0, 180);
        auto img2x = circShift(img2.img, 0, 180);

        // Step 2: Perform FFT-based template matching for the shifted orientation
        auto secondRect = fftMatch(img2x, img1.img);
        int secondShift = secondRect.center.x - img1.img.cols / 2;

        // Step 3: Compute the Hamming distance for the shifted orientation
        float dis2 = 0;
        int bias2 = 0;
        getHammingDistance(img1.T, img1.M, T2x, M2x, secondShift, dis2, bias2);

        // Step 4: Return the Hamming distance and corresponding bias
        if (bias)
            *bias = (bias2 + 180) % 360;  // Adjust the bias for the 180-degree shift
        return dis2;
    }

    // If match_mode_ == 0, only compare the original orientation
    if (match_mode_ == 0)
    {
        // Step 1: Perform FFT-based template matching for the original orientation
        auto firstRect = fftMatch(img2.img, img1.img);
        int firstShift = firstRect.center.x - img1.img.cols / 2;

        // Step 2: Compute the Hamming distance for the original orientation
        float dis1;
        int bias1;
        getHammingDistance(img1.T, img1.M, img2.T, img2.M, firstShift, dis1, bias1);

        // Step 3: Return the Hamming distance and corresponding bias
        if (bias)
            *bias = bias1;
        return dis1;
    }
}


// ==============================================================================================
// Functions for main distributed mapping

