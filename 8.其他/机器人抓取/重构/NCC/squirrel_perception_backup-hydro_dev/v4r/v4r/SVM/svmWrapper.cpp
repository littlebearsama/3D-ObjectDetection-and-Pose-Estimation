#include "svmWrapper.h"


bool myfunction (data i,data j) { return (i.y<j.y); }

namespace svm
{
    void svmWrapper::initSVM()
    {
        svm_para_.svm_type = svm::C_SVC;
        svm_para_.kernel_type = svm::RBF;
        //svm_para->degree = 2;
        svm_para_.gamma = 0.01;// default 1/k;
        //svm_para->coef0 = 1;

        svm_para_.cache_size = 100;
        svm_para_.eps = 0.001;
        svm_para_.C = 10;
        svm_para_.nr_weight = 0;
        svm_para_.weight_label = NULL;
        svm_para_.weight = NULL;
        //svm_para->nu = 0.5;
        //svm_para->p = 1;
        svm_para_.shrinking = 1;
        svm_para_.probability = 1;
    }

    void svmWrapper::computeConfusionMatrix( const std::vector<double> &target_pred,
                                               const std::vector<double> &target_test,
                                               cv::Mat &confusion_matrix)
    {
        if(target_pred.size() != target_test.size())
        {
            std::cerr << "The target vectors of the predicted and actual classes do not have the same size."
                       <<  "Cannot calculate confusion matrix... " << std::endl;
        }
        else
        {
            confusion_matrix = cv::Mat::zeros(num_classes_, num_classes_, CV_16U);

            for(size_t i=0; i< target_pred.size(); i++)
            {
                confusion_matrix.at<unsigned short>(target_test[i], target_pred[i]) ++;
            }
        }
    }


    void svmWrapper::SVMpredictTarget( const std::vector<std::vector<double> > & data_test,
                                         std::vector<double> &target_pred)
    {
        target_pred.resize (data_test.size());

        for(size_t i=0; i<data_test.size(); i++)
        {
            svm::svm_node *svm_n_test = new svm::svm_node[data_test[i].size()+1];

            for(size_t kk=0; kk<data_test[i].size(); kk++)
            {
                svm_n_test[kk].value = data_test[i][kk];
                svm_n_test[kk].index = kk+1;
            }
            svm_n_test[data_test[i].size()].index = -1;
            target_pred[i] = svm::svm_predict(svm_mod_, svm_n_test);
        }
    }


    void svmWrapper::testSVM(const std::vector<std::vector<double> > & data_test,
                                 const std::vector<double> &target_actual,
                                 std::vector<double> &target_pred,
                                 cv::Mat &confusion_matrix)
    {
        size_t num_falsely_classified=0, num_correctly_classified=0;

        target_pred.resize (data_test.size());

        for(size_t i=0; i<data_test.size(); i++)
        {
            svm::svm_node *svm_n_test = new svm::svm_node[data_test[i].size()+1];

            for(size_t kk=0; kk<data_test[i].size(); kk++)
            {
                svm_n_test[kk].value = data_test[i][kk];
                svm_n_test[kk].index = kk+1;
            }
            svm_n_test[data_test[i].size()].index = -1;
            double prob[num_classes_];
            target_pred[i] = svm::svm_predict_probability(svm_mod_, svm_n_test, prob);

            if(target_actual.size()>i)
            {
                if(target_pred[i] == target_actual[i])
                {
                    num_correctly_classified++;
                }
                else
                {
                    num_falsely_classified++;
                }
            }
        }
        computeConfusionMatrix(target_pred, target_actual, confusion_matrix);
    }

    void svmWrapper::dokFoldCrossValidation(
            const std::vector<std::vector<double> > &data_train,
            const std::vector<double> &target_train,
            size_t k,
            double model_para_C_min,
            double model_para_C_max,
            double step_multiplicator_C,
            double model_para_gamma_min,
            double model_para_gamma_max,
            double step_multiplicator_gamma)
    {
        double bestC = model_para_C_min, bestGamma = model_para_gamma_min, bestTestPerformanceValue=0;
        std::vector<cv::Mat> best_confusion_matrices_v(k);
        std::vector<cv::Mat> confusion_matrices_v(k);

        for(double C = model_para_C_min; C <= model_para_C_max; C *= step_multiplicator_C)
        {
            for(double gamma = model_para_gamma_min; gamma <= model_para_gamma_max; gamma *= step_multiplicator_gamma)
            {
                double avg_performance;
                svm_para_.C = C;
                svm_para_.gamma = gamma;
                std::cout << "Computing svm for C=" << C << " and gamma=" << gamma << std::endl;

                for(size_t current_val_set_id = 0; current_val_set_id < k; current_val_set_id++)
                {
                    std::vector<std::vector<double> > data_train_sub;
                    std::vector<std::vector<double> > data_val;
                    std::vector<double> target_train_sub;
                    std::vector<double> target_val;

                    for(size_t i=0; i < target_train.size(); i++)
                    {
                        if(i%k == current_val_set_id)
                        {
                            data_val.push_back(data_train[i]);
                            target_val.push_back(target_train[i]);
                        }
                        else
                        {
                            data_train_sub.push_back(data_train[i]);
                            target_train_sub.push_back(target_train[i]);
                        }
                    }
                    computeSvmModel(data_train_sub, target_train_sub);
                    std::vector<double> target_pred;
                    testSVM(data_val, target_val, target_pred, confusion_matrices_v[current_val_set_id]);
                    std::cout << "confusion matrix ( " << current_val_set_id << ")" << std::endl << confusion_matrices_v[current_val_set_id] << std::endl;
                }

                cv::Mat total_confusion_matrix = cv::Mat::zeros(num_classes_, num_classes_, CV_16U);
                for(size_t i=0; i< k; i++)
                {
                    total_confusion_matrix += confusion_matrices_v[i];
                }
                std::cout << "Total confusion matrix:" << std::endl << total_confusion_matrix << std::endl << std::endl;

                size_t sum=0;
                size_t trace=0;
                for(int i=0; i<total_confusion_matrix.rows; i++)
                {
                    for(int jj=0; jj<total_confusion_matrix.cols; jj++)
                    {
                        sum += total_confusion_matrix.at<unsigned short>(i,jj);
                        if (i == jj)
                            trace += total_confusion_matrix.at<unsigned short>(i,jj);
                    }
                }
                avg_performance = static_cast<double>(trace) / sum;

                std::cout << "My average performance is " << avg_performance << std::endl;

                if(avg_performance > bestTestPerformanceValue)
                {
                    bestTestPerformanceValue = avg_performance;
                    bestC = C;
                    bestGamma = gamma;
                    for(size_t i=0; i<k; i++)
                    {
                        best_confusion_matrices_v[i] = confusion_matrices_v[i].clone();
                        std::cout << "best confusion matrix ( " << i << ")" << std::endl << best_confusion_matrices_v[i] << std::endl;
                    }
                }
            }
        }
        svm_para_.C = bestC;
        svm_para_.gamma = bestGamma;

        cv::Mat confusion_matrix = cv::Mat::zeros(num_classes_, num_classes_, CV_16U);
        for(size_t i=0; i< k; i++)
        {
            confusion_matrix += best_confusion_matrices_v[i];
            std::cout << "Confusion matrix (part " << i << "/" << k << "): " << std::endl << best_confusion_matrices_v[i] << std::endl << std::endl;
        }
        std::cout << "I achieved the best performance(" << bestTestPerformanceValue<< ") for C=" << bestC <<
                     " and gamma=" << bestGamma << ". " << std::endl <<
                     "Total confusion matrix:" << std::endl << confusion_matrix << std::endl << std::endl;
    }

    void svmWrapper::computeSvmModel(
            const std::vector<std::vector<double> > &data_train,
            const std::vector<double> &target_train)
    {
        svm::svm_problem *svm_prob = new svm::svm_problem;

        svm_prob->l = data_train.size(); //number of training examples
        svm_prob->x = new svm::svm_node *[svm_prob->l];
        for(int i = 0; i<svm_prob->l; i++)
            svm_prob->x[i] = new svm::svm_node[data_train[i].size()+1];  // add one additional dimension and set that index to -1 (libsvm requirement)
        svm_prob->y = new double[svm_prob->l];

        for(int i=0; i<svm_prob->l; i++)
        {
            for(size_t kk=0; kk<data_train[i].size(); kk++)
            {
                svm_prob->x[i][kk].value = data_train[i][kk];
                svm_prob->x[i][kk].index = kk+1;
                svm_prob->y[i] = target_train[i];
            }
            svm_prob->x[i][data_train[i].size()].index = -1;
        }
        svm_mod_ = svm::svm_train(svm_prob, &svm_para_);
    }

    void svmWrapper::shuffleTrainingData(std::vector<std::vector<double> > &data_train, std::vector<double> &target_train)
    {
        std::vector<size_t> vector_indices;
        vector_indices.reserve(data_train.size());
        for(size_t i=0; i<data_train.size(); i++)
            vector_indices.push_back(i);
        std::random_shuffle(vector_indices.begin(), vector_indices.end());
        for(size_t i=0; i<data_train.size(); i++)
        {
            std::swap(data_train[i], data_train[vector_indices[i]]);
            std::swap(target_train[i], target_train[vector_indices[i]]);
        }
    }

    void svmWrapper::sortTrainingData(
            std::vector<std::vector<double> > &data_train,
            std::vector<double> &target_train)
    {
        std::vector<data> d(data_train.size());
        for(size_t i=0; i<data_train.size(); i++)
        {
            d[i].x = data_train[i];
            d[i].y = target_train[i];
        }
        std::sort(d.begin(),d.end(),myfunction);
        for(size_t i=0; i<data_train.size(); i++)
        {
            data_train[i] = d[i].x;
            target_train[i] = d[i].y;
        }
    }
}
