/*
Copyright (c) 2015, Kripasindhu Sarkar
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder(s) nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


 Original code under the following Apache License 2.0:
 Changes - None

 ----------
  This software is licensed under the Apache License 2.0 (http://www.apache.org/licenses/LICENSE-2.0.html) Please find the Apache license 2.0 statement in the respective file alongside this software.

Copyright 2015 Jan Hendriks

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
 ----------
 */

/**
 * @file:   svmlight.h
 * @author: Jan Hendriks (dahoc3150 [at] gmail.com)
 * @date:   Created on 11. Mai 2011
 * @brief:  Wrapper interface for SVMlight, 
 * @see http://www.cs.cornell.edu/people/tj/svm_light/ for SVMlight details and terms of use
 * 
 */

#ifndef SVMLIGHT_H
#define	SVMLIGHT_H

#include <stdio.h>
#include <vector>
// svmlight related
// namespace required for avoiding collisions of declarations (e.g. LINEAR being declared in flann, svmlight and libsvm)
namespace svmlight {
    extern "C" {
        #include "3rdparty/svmlight/svm_common.h"
        #include "3rdparty/svmlight/svm_learn.h"
    }
}

using namespace svmlight;

class SVMlight {
private:
    DOC** docs; // training examples
    long totwords, totdoc, i; // support vector stuff
    double* target;
    double* alpha_in;
    KERNEL_CACHE* kernel_cache;
    MODEL* model; // SVM model

    SVMlight() {
        // Init variables
        alpha_in = NULL;
        kernel_cache = NULL; // Cache not needed with linear kernel
        model = (MODEL *) my_malloc(sizeof (MODEL));
        learn_parm = new LEARN_PARM;
        kernel_parm = new KERNEL_PARM;
        // Init parameters
        verbosity = 1; // Show some messages -v 1
        learn_parm->alphafile[0] = ' '; // NULL; // Important, otherwise files with strange/invalid names appear in the working directory
        //        learn_parm->alphafile = NULL; // Important, otherwise files with strange/invalid names appear in the working directory
        learn_parm->biased_hyperplane = 1;
        learn_parm->sharedslack = 0; // 1
        learn_parm->skip_final_opt_check = 0;
        learn_parm->svm_maxqpsize = 10;
        learn_parm->svm_newvarsinqp = 0;
        learn_parm->svm_iter_to_shrink = 2; // 2 is for linear;
        learn_parm->kernel_cache_size = 40;
        learn_parm->maxiter = 100000;
        learn_parm->svm_costratio = 1.0;
        learn_parm->svm_costratio_unlab = 1.0;
        learn_parm->svm_unlabbound = 1E-5;
        learn_parm->eps = 0.1;
        learn_parm->transduction_posratio = -1.0;
        learn_parm->epsilon_crit = 0.001;
        learn_parm->epsilon_a = 1E-15;
        learn_parm->compute_loo = 0;
        learn_parm->rho = 1.0;
        learn_parm->xa_depth = 0;
        // The HOG paper uses a soft classifier (C = 0.01), set to 0.0 to get the default calculation
        learn_parm->svm_c = 0.01; // -c 0.01
        learn_parm->type = REGRESSION;
        learn_parm->remove_inconsistent = 0; // -i 0 - Important
        kernel_parm->rbf_gamma = 1.0;
        kernel_parm->coef_lin = 1;
        kernel_parm->coef_const = 1;
        kernel_parm->kernel_type = LINEAR; // -t 0
        kernel_parm->poly_degree = 3;
    }

    virtual ~SVMlight() {
        // Cleanup area
        // Free the memory used for the cache
        if (kernel_cache)
            kernel_cache_cleanup(kernel_cache);
        free(alpha_in);
        free_model(model, 0);
        for (i = 0; i < totdoc; i++)
            free_example(docs[i], 1);
        free(docs);
        free(target);
    }

public:
    LEARN_PARM* learn_parm;
    KERNEL_PARM* kernel_parm;

    static SVMlight* getInstance();

    inline void saveModelToFile(const std::string _modelFileName) {
        write_model(const_cast<char*>(_modelFileName.c_str()), model);
    }

    void loadModelFromFile(const std::string _modelFileName) {
        this->model = read_model(const_cast<char*>(_modelFileName.c_str()));
    }

    // read in a problem (in svmlight format)
    void read_problem(char* filename) {
        // Reads and parses the specified file
        read_documents(filename, &docs, &target, &totwords, &totdoc);
    }

    // Calls the actual machine learning algorithm
    void train() {
        svm_learn_regression(docs, target, totdoc, totwords, learn_parm, kernel_parm, &kernel_cache, model);
    }

    /**
     * Generates a single detecting feature vector (vec1) from the trained support vectors, for use e.g. with the HOG algorithm
     * vec1 = sum_1_n (alpha_y*x_i). (vec1 is a 1 x n column vector. n = feature vector length)
     * @param singleDetectorVector resulting single detector vector for use in openCV HOG
     * @param singleDetectorVectorIndices dummy vector for this implementation
     */
    void getSingleDetectingVector(std::vector<float>& singleDetectorVector, std::vector<unsigned int>& singleDetectorVectorIndices) {
        // Now we use the trained svm to retrieve the single detector vector
        DOC** supveclist = model->supvec;
        printf("Calculating single descriptor vector out of support vectors (may take some time)\n");
        // Retrieve single detecting vector (v1) from returned ones by calculating vec1 = sum_1_n (alpha_y*x_i). (vec1 is a n x1 column vector. n = feature vector length)
        singleDetectorVector.clear();
        singleDetectorVector.resize(model->totwords, 0.);
        printf("Resulting vector size %lu\n", singleDetectorVector.size());
        
        // Walk over every support vector
        for (long ssv = 1; ssv < model->sv_num; ++ssv) { // Don't know what's inside model->supvec[0] ?!
            // Get a single support vector
            DOC* singleSupportVector = supveclist[ssv]; // Get next support vector
            SVECTOR* singleSupportVectorValues = singleSupportVector->fvec;
            WORD singleSupportVectorComponent;
            // Walk through components of the support vector and populate our detector vector
            for (unsigned long singleFeature = 0; singleFeature < model->totwords; ++singleFeature) {
                singleSupportVectorComponent = singleSupportVectorValues->words[singleFeature];
                singleDetectorVector.at(singleSupportVectorComponent.wnum-1) += (singleSupportVectorComponent.weight * model->alpha[ssv]);
            }
        }
    }
    
    /**
     * Return model detection threshold / bias
     * @return detection threshold / bias
     */
    float getThreshold() const {
        return model->b;
    }
    
    const char* getSVMName() const {
        return "SVMlight";
    }

};

/// Singleton
SVMlight* SVMlight::getInstance() {
    static SVMlight theInstance;
    return &theInstance;
}

#endif	/* SVMLIGHT_H */
