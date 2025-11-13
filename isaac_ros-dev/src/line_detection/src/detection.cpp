
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
    else {
        gray_img = image;
    }
    int height = gray_img.rows;
    int width = gray_img.cols;
    RCLCPP_INFO(rclcpp::get_logger("lines"), "input image r x c : %d x %d ", height, width);


    // get mask
    cv::Mat mask;
    double threshold = 200;
    cv::threshold(gray_img, mask, threshold, 255, cv::THRESH_BINARY);

    

    // Npp32u is typedef unsigned int. which is pretty much uint32_t. So I'm not casting it.

    std::pair<Npp32f *, Npp64f *> integrals = __get_integral_image(gray_img);

    Npp32f * integral;
    Npp64f * integral_sq;

    std::tie(integral, integral_sq) = integrals;
    

    // allocate memory for CERIAS
    // float * gray, np32f integral, the square, uint8_t * mask, int2 * output, int * counter (output len)

    cv::Mat gray_float;
    gray_img.convertTo(gray_float, CV_32F);

    float * input_image_device;
    size_t total = width * height;
    
    HANDLE_ERROR( cudaMalloc((void**) &input_image_device, total * sizeof(float)) );
    HANDLE_ERROR( cudaMemcpy(input_image_device, gray_float.ptr<float>(),total * sizeof(float), cudaMemcpyHostToDevice ));

    // integral and integral sq are already on device

    // mask allocation
    uint8_t *device_mask;
    size_t mask_size = total * sizeof(uint8_t);
    HANDLE_ERROR(cudaMalloc((void**)&device_mask, mask_size));
    HANDLE_ERROR(cudaMemcpy(device_mask, mask.ptr<uint8_t>(), mask_size, cudaMemcpyHostToDevice));

    int2 * output;
 
    HANDLE_ERROR( cudaMalloc((void**) &output, total * sizeof(int2)) );
    HANDLE_ERROR( cudaMemset(output, 0, total * sizeof(int2)));

    int * counter;
 
    HANDLE_ERROR( cudaMalloc((void**) &counter, sizeof(int)) );
    HANDLE_ERROR( cudaMemset(counter, 0, sizeof(int)));

    // finally...

    cerias_kernel(
        input_image_device,
        integral, integral_sq,
        device_mask,
        output, counter,
        width, height
    );



    // going to try the direct ros topic mapping. If not, I'll be back to memcopy output and counter

    // clean up and return
    cudaFree(input_image_device);
    cudaFree(integral);
    cudaFree(integral_sq);
    cudaFree(device_mask);
    //cudaFree(counter);



    //return output;
    int *counter_return = new int;
    HANDLE_ERROR( cudaMemcpy(counter_return, counter, sizeof(int), cudaMemcpyDeviceToHost) );

    int2 *output_return = new int2[*counter_return];
    HANDLE_ERROR( cudaMemcpy(output_return, output, *counter_return * sizeof(int2), cudaMemcpyDeviceToHost) );

    cudaDeviceSynchronize();


    return std::make_pair(output_return, counter_return);

    // deallocate in main
    


}

/**
 * Retreives the integral image and square integral image from a grayscale cv2 image
 * 
 * RETURNS DEVICE POINTERS.... this function is meant to used internally and keep the data on device for downstream processing
 * do NOT call this function and then dereference anything without memcpy-ing back to host.
 * 
 */
std::pair<Npp32f *, Npp64f *> __get_integral_image(const cv::Mat &gray_img) {

    int width = gray_img.cols;
    int height = gray_img.rows;

    // allocate memory for Integral 

    Npp8u *device_input_img;

    size_t image_size = gray_img.rows * gray_img.cols * sizeof(Npp8u);
    HANDLE_ERROR( cudaMalloc(&device_input_img, image_size) ) ;
    // CAREFUL, WE ARE CASTING TO 8 BIT PIXELS HERE, MAKE SURE INPUT IS 8 BIT
    HANDLE_ERROR( cudaMemcpy(device_input_img, gray_img.ptr<Npp8u>(), image_size, cudaMemcpyHostToDevice) );

    Npp32f *result;
    Npp64f *result_sq;
    
    size_t result_size = (height + 1) * (width + 1) * sizeof(Npp32f);
    HANDLE_ERROR( cudaMalloc(&result, result_size) );
    HANDLE_ERROR( cudaMemset(result, 0, result_size) );

    size_t result_sq_size = (height + 1) * (width + 1) * sizeof(Npp64f);
    HANDLE_ERROR( cudaMalloc(&result_sq, result_sq_size) );
    HANDLE_ERROR( cudaMemset(result_sq, 0, result_sq_size) );

    // set nsrcstep, ndststep, and roi

    size_t nsrcstep = width * sizeof(Npp8u);
    size_t ndststep = (width + 1) * sizeof(Npp32f);
    size_t nsqrstep = (width + 1) * sizeof(Npp64f);
    NppiSize roi = { width, height };

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
        std::cerr << "integral did not work. Your error code is: " << status << std::endl;
        exit( EXIT_FAILURE );
    }
    cudaFree(device_input_img);

    return std::make_pair(result, result_sq);



}




