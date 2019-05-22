#ifndef SVMWRAPPER_H
#define SVMWRAPPER_H

#include "svm.h"
#include <vector>
//#include <cv.h>
#include <opencv/cv.h>

struct data
{
    std::vector<double> x;
    double y;
};


namespace svm
{

    class svmWrapper
    {
    private:
        size_t num_classes_;

    public:
        svm::svm_parameter svm_para_;
        svm::svm_model  *svm_mod_;

        svmWrapper()
        {
            num_classes_ = 0;
        }

        void SVMpredictTarget(const std::vector<std::vector<double> > & data_test,
                                     std::vector<double> &target_pred);

        void testSVM(
                const std::vector<std::vector<double> > & data_test,
                const std::vector<double> &target_actual,
                std::vector<double> &target_pred,
                cv::Mat &confusion_matrix);


        void dokFoldCrossValidation(
                const std::vector<std::vector<double> > &data_train,
                const std::vector<double> &target_train,
                size_t k,
                double model_para_C_min = exp2(-6),
                double model_para_C_max = exp2(6),
                double step_multiplicator_C = 2,
                double model_para_gamma_min = exp(-5),
                double model_para_gamma_max = exp(5),
                double step_multiplicator_gamma = 2);

        void shuffleTrainingData(
                std::vector<std::vector<double> > &data_train,
                std::vector<double> &target_train);

        void computeConfusionMatrix( const std::vector<double> &target_pred,
                                                     const std::vector<double> &target_test,
                                                     cv::Mat &confusion_matrix);
        void initSVM();

        void computeSvmModel(
                const std::vector<std::vector<double> > &data_train,
                const std::vector<double> &target_train);

        void setNumClasses(const size_t num_classes)
        {
            num_classes_ =  num_classes;
        }

        void sortTrainingData(
                std::vector<std::vector<double> > &data_train,
                std::vector<double> &target_train);
};

}
#endif //SVMWRAPPER_H
