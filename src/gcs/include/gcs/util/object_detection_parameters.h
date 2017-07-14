
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
    
    struct ObjectDetectionParameters
    {
        ObjectDetectionParameters& operator=(const ObjectDetectionParameters& other)
        {
            this->hit_thresh = other.hit_thresh;
            this->step_size = other.step_size;
            this->padding = other.padding;
            this->scale_factor = other.scale_factor;
            this->mean_shift = other.mean_shift;
            
            return *this;
        }
        
        //defaults
        double hit_thresh; // displayed as a decimal
        int step_size;
        int padding;
        double scale_factor; // displayed as a decimal
        bool mean_shift;
    };
    
}

#endif /* OBJECT_DETECTION_PARAMETERS_H */

