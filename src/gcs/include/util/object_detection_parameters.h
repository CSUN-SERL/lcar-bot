
/* 
 * File:   object_detection_parameters.h
 * Author: n8
 *
 * Created on November 18, 2016, 12:06 PM
 */

#ifndef OBJECT_DETECTION_PARAMETERS_H
#define OBJECT_DETECTION_PARAMETERS_H

namespace gcs
{
    
    struct ObjectDetectionParameters_
    {
        ObjectDetectionParameters_& operator=(const ObjectDetectionParameters_& other)
        {
            this->hit_thresh = other.hit_thresh;
            this->step_size = other.step_size;
            this->padding = other.padding;
            this->scale_factor = other.scale_factor;
            this->mean_shift = false;
            
            return *this;
        }
        
        //defaults
        double hit_thresh = 0; // displayed as a decimal
        int step_size = 16;
        int padding = 8;
        double scale_factor = 1.15; // displayed as a decimal
        bool mean_shift = false;
    };
    
    typedef ObjectDetectionParameters_ ObjectDetectionParameters;

}

#endif /* OBJECT_DETECTION_PARAMETERS_H */

