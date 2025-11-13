#include "line_detection/detection.hpp"

// Error function and macro borrowed from 
// https://github.com/jiekebo/CUDA-By-Example/blob/master/common/book.h

// thank you 

/**
 * Detects and returns pixels likely belonging to lines. 
 * Most noise detected is either far outside the course, or is part of an obstacle.
 * Thus, we are happy to map these pixels as obstacles on the map, since the robot will avoid them anyways. 
 * 
 */
std::pair<int2*, int*> lines::detect_line_pixels(const cv::Mat &image) {

    // convert to grayscale
    cv::Mat gray_img;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray_img, cv::COLOR_BGR2GRAY);
    }
    else if (image.channels() == 4) {
        cv::cvtColor(image, gray_img, cv::COLOR_BGRA2GRAY);
    }
    else {
        gray_img = image;
    }
    
    int height = gray_img.rows;
    int width = gray_img.cols;
    RCLCPP_INFO(rclcpp::get_logger("lines"), "input image r x c : %d x %d ", height, width);

    // Validate image dimensions
    if (height <= 0 || width <= 0 || height > 10000 || width > 10000) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "Invalid image dimensions: %dx%d", height, width);
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    // Validate image data
    if (gray_img.data == nullptr || gray_img.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "Empty image data");
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    // Validate continuous data
    if (!gray_img.isContinuous()) {
        RCLCPP_WARN(rclcpp::get_logger("lines"), "Image not continuous, making copy");
        gray_img = gray_img.clone();
    }

    // get mask
    cv::Mat mask;
    double threshold = 200;
    cv::threshold(gray_img, mask, threshold, 255, cv::THRESH_BINARY);

    RCLCPP_INFO(rclcpp::get_logger("lines"), "Threshold complete, computing integral images");

    // Get integral images
    std::pair<Npp32f *, Npp64f *> integrals;
    try {
        integrals = __get_integral_image(gray_img);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "Integral image failed: %s", e.what());
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    Npp32f * integral;
    Npp64f * integral_sq;
    std::tie(integral, integral_sq) = integrals;
    
    RCLCPP_INFO(rclcpp::get_logger("lines"), "Integral images computed, allocating CUDA memory");

    // allocate memory for CERIAS
    cv::Mat gray_float;
    gray_img.convertTo(gray_float, CV_32F);

    // Validate conversion
    if (gray_float.empty() || gray_float.data == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "Float conversion failed");
        cudaFree(integral);
        cudaFree(integral_sq);
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    float * input_image_device;
    size_t total = width * height;
    
    RCLCPP_INFO(rclcpp::get_logger("lines"), "Allocating device memory for %zu pixels", total);
    
    cudaError_t err;
    err = cudaMalloc((void**) &input_image_device, total * sizeof(float));
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "cudaMalloc failed for input image: %s", 
                     cudaGetErrorString(err));
        cudaFree(integral);
        cudaFree(integral_sq);
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }
    
    err = cudaMemcpy(input_image_device, gray_float.ptr<float>(), total * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "cudaMemcpy failed for input image: %s", 
                     cudaGetErrorString(err));
        cudaFree(input_image_device);
        cudaFree(integral);
        cudaFree(integral_sq);
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    RCLCPP_INFO(rclcpp::get_logger("lines"), "Input image copied to device");

    // mask allocation
    uint8_t *device_mask;
    size_t mask_size = total * sizeof(uint8_t);
    
    err = cudaMalloc((void**)&device_mask, mask_size);
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "cudaMalloc failed for mask: %s", 
                     cudaGetErrorString(err));
        cudaFree(input_image_device);
        cudaFree(integral);
        cudaFree(integral_sq);
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }
    
    err = cudaMemcpy(device_mask, mask.ptr<uint8_t>(), mask_size, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "cudaMemcpy failed for mask: %s", 
                     cudaGetErrorString(err));
        cudaFree(input_image_device);
        cudaFree(device_mask);
        cudaFree(integral);
        cudaFree(integral_sq);
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    RCLCPP_INFO(rclcpp::get_logger("lines"), "Mask copied to device");

    int2 * output;
    err = cudaMalloc((void**) &output, total * sizeof(int2));
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "cudaMalloc failed for output: %s", 
                     cudaGetErrorString(err));
        cudaFree(input_image_device);
        cudaFree(device_mask);
        cudaFree(integral);
        cudaFree(integral_sq);
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }
    
    err = cudaMemset(output, 0, total * sizeof(int2));
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "cudaMemset failed for output: %s", 
                     cudaGetErrorString(err));
        cudaFree(input_image_device);
        cudaFree(device_mask);
        cudaFree(output);
        cudaFree(integral);
        cudaFree(integral_sq);
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    int * counter;
    err = cudaMalloc((void**) &counter, sizeof(int));
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "cudaMalloc failed for counter: %s", 
                     cudaGetErrorString(err));
        cudaFree(input_image_device);
        cudaFree(device_mask);
        cudaFree(output);
        cudaFree(integral);
        cudaFree(integral_sq);
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }
    
    err = cudaMemset(counter, 0, sizeof(int));
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "cudaMemset failed for counter: %s", 
                     cudaGetErrorString(err));
        cudaFree(input_image_device);
        cudaFree(device_mask);
        cudaFree(output);
        cudaFree(counter);
        cudaFree(integral);
        cudaFree(integral_sq);
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    RCLCPP_INFO(rclcpp::get_logger("lines"), "All device memory allocated, launching kernel");

    // Launch kernel
    cerias_kernel(
        input_image_device,
        integral, integral_sq,
        device_mask,
        output, counter,
        width, height
    );

    // Check for kernel launch errors
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "Kernel launch failed: %s", 
                     cudaGetErrorString(err));
        cudaFree(input_image_device);
        cudaFree(device_mask);
        cudaFree(output);
        cudaFree(counter);
        cudaFree(integral);
        cudaFree(integral_sq);
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    // Wait for kernel to complete
    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "Kernel execution failed: %s", 
                     cudaGetErrorString(err));
        cudaFree(input_image_device);
        cudaFree(device_mask);
        cudaFree(output);
        cudaFree(counter);
        cudaFree(integral);
        cudaFree(integral_sq);
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    RCLCPP_INFO(rclcpp::get_logger("lines"), "Kernel executed successfully");

    // Copy results back
    int *counter_return = new int;
    err = cudaMemcpy(counter_return, counter, sizeof(int), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "cudaMemcpy failed for counter: %s", 
                     cudaGetErrorString(err));
        delete counter_return;
        cudaFree(input_image_device);
        cudaFree(device_mask);
        cudaFree(output);
        cudaFree(counter);
        cudaFree(integral);
        cudaFree(integral_sq);
        counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    RCLCPP_INFO(rclcpp::get_logger("lines"), "Detected %d line pixels", *counter_return);

    // Allocate output array based on actual count
    int2 *output_return = new int2[std::max(1, *counter_return)];
    
    if (*counter_return > 0) {
        err = cudaMemcpy(output_return, output, *counter_return * sizeof(int2), cudaMemcpyDeviceToHost);
        if (err != cudaSuccess) {
            RCLCPP_ERROR(rclcpp::get_logger("lines"), "cudaMemcpy failed for output: %s", 
                         cudaGetErrorString(err));
            delete[] output_return;
            delete counter_return;
            cudaFree(input_image_device);
            cudaFree(device_mask);
            cudaFree(output);
            cudaFree(counter);
            cudaFree(integral);
            cudaFree(integral_sq);
            counter_return = new int;
            *counter_return = 0;
            output_return = new int2[1];
            return std::make_pair(output_return, counter_return);
        }
    }

    // Clean up device memory
    cudaFree(input_image_device);
    cudaFree(integral);
    cudaFree(integral_sq);
    cudaFree(device_mask);
    cudaFree(output);
    cudaFree(counter);

    RCLCPP_INFO(rclcpp::get_logger("lines"), "Cleanup complete, returning results");

    return std::make_pair(output_return, counter_return);
}

/**
 * Retrieves the integral image and square integral image from a grayscale cv2 image
 * 
 * RETURNS DEVICE POINTERS.... this function is meant to used internally and keep the data on device for downstream processing
 * do NOT call this function and then dereference anything without memcpy-ing back to host.
 * 
 */
std::pair<Npp32f *, Npp64f *> __get_integral_image(const cv::Mat &gray_img) {

    int width = gray_img.cols;
    int height = gray_img.rows;

    RCLCPP_INFO(rclcpp::get_logger("lines"), "Computing integral image for %dx%d image", width, height);

    // Validate dimensions
    if (width <= 0 || height <= 0 || width > 10000 || height > 10000) {
        throw std::runtime_error("Invalid image dimensions for integral image");
    }

    // allocate memory for Integral 
    Npp8u *device_input_img;

    size_t image_size = gray_img.rows * gray_img.cols * sizeof(Npp8u);
    
    cudaError_t err = cudaMalloc(&device_input_img, image_size);
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("cudaMalloc failed: ") + cudaGetErrorString(err));
    }
    
    // CAREFUL, WE ARE CASTING TO 8 BIT PIXELS HERE, MAKE SURE INPUT IS 8 BIT
    if (gray_img.type() != CV_8UC1) {
        cudaFree(device_input_img);
        throw std::runtime_error("Input image must be CV_8UC1");
    }
    
    err = cudaMemcpy(device_input_img, gray_img.ptr<Npp8u>(), image_size, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        cudaFree(device_input_img);
        throw std::runtime_error(std::string("cudaMemcpy failed: ") + cudaGetErrorString(err));
    }

    Npp32f *result;
    Npp64f *result_sq;
    
    size_t result_size = (height + 1) * (width + 1) * sizeof(Npp32f);
    err = cudaMalloc(&result, result_size);
    if (err != cudaSuccess) {
        cudaFree(device_input_img);
        throw std::runtime_error(std::string("cudaMalloc failed for result: ") + cudaGetErrorString(err));
    }
    
    err = cudaMemset(result, 0, result_size);
    if (err != cudaSuccess) {
        cudaFree(device_input_img);
        cudaFree(result);
        throw std::runtime_error(std::string("cudaMemset failed: ") + cudaGetErrorString(err));
    }

    size_t result_sq_size = (height + 1) * (width + 1) * sizeof(Npp64f);
    err = cudaMalloc(&result_sq, result_sq_size);
    if (err != cudaSuccess) {
        cudaFree(device_input_img);
        cudaFree(result);
        throw std::runtime_error(std::string("cudaMalloc failed for result_sq: ") + cudaGetErrorString(err));
    }
    
    err = cudaMemset(result_sq, 0, result_sq_size);
    if (err != cudaSuccess) {
        cudaFree(device_input_img);
        cudaFree(result);
        cudaFree(result_sq);
        throw std::runtime_error(std::string("cudaMemset failed: ") + cudaGetErrorString(err));
    }

    // set nsrcstep, ndststep, and roi
    size_t nsrcstep = width * sizeof(Npp8u);
    size_t ndststep = (width + 1) * sizeof(Npp32f);
    size_t nsqrstep = (width + 1) * sizeof(Npp64f);
    NppiSize roi = { width, height };

    RCLCPP_INFO(rclcpp::get_logger("lines"), "Calling nppiSqrIntegral_8u32f64f_C1R");

    // take npp integral
    NppStatus status;
    status = nppiSqrIntegral_8u32f64f_C1R(
        device_input_img, // input pointer (device)
        nsrcstep, // row length input
        result,  // result pointer (device)
        ndststep, // row length result 
        result_sq, // square result pointer
        nsqrstep, // square result row size
        roi, // width and height
        0, // 0 and dont question it
        0 // see above
    );

    if (status != NPP_SUCCESS) {
        cudaFree(device_input_img);
        cudaFree(result);
        cudaFree(result_sq);
        std::string error_msg = "nppiSqrIntegral failed with code: " + std::to_string(status);
        throw std::runtime_error(error_msg);
    }
    
    cudaFree(device_input_img);

    RCLCPP_INFO(rclcpp::get_logger("lines"), "Integral image computed successfully");

    return std::make_pair(result, result_sq);
}