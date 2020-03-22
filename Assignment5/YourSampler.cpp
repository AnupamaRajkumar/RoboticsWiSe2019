#include <chrono>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/VectorPtr.h>
#include <iostream>
#include "YourSampler.h"

namespace rl
{
    namespace plan
    {
        YourSampler::YourSampler() :
            Sampler(),
            randDistribution(0, 1),
            gaussDistribution(0,1),
            gaussEngine(::std::random_device()()),
            randEngine(::std::random_device()())
        {
        }

        YourSampler::~YourSampler()
        {
        }

        ::rl::math::Vector
        YourSampler::generate()
        {
            ::rl::math::Vector rand(this->model->getDof());

            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                rand(i) = this->rand();
            }

            return this->model->generatePositionUniform(rand);
        }

        ::rl::math::Vector
        YourSampler::generateCollisionFree()
        {
            ::rl::math::Vector q(this->model->getDof());
            ::rl::math::Vector q2(this->model->getDof());
            ::rl::math::Vector sigma(this->model->getDof());
            ::rl::math::Vector chosen(this->model->getDof());
            ::rl::math::Real distance;

            ::rl::plan::VectorPtr last;

            while(true)
           {
                q = this->generate();

                last = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());
                ::rl::math::Vector gauss(this->model->getDof());

                for(::std::size_t i = 0; i < this->model->getDof(); i++)
                {
                    gauss(i) = this->gauss();
                    sigma(i) = 2.0;
                }

                q2 = this->model->generatePositionGaussian(gauss, q, sigma);

                /*collision check*/
               this->model->setPosition(q);
               this->model->updateFrames();

               if (!this->model->isColliding())
               {
                   this->model->setPosition(q2);
                   this->model->updateFrames();

                   if (this->model->isColliding())
                   {
                       return q;
                   }
               }
               else
               {
                   this->model->setPosition(q2);
                   this->model->updateFrames();

                   if (!this->model->isColliding())
                   {
                       q = q2;
                       return q;
                   }
               }
            }

        }

        ::std::uniform_real_distribution< ::rl::math::Real>::result_type
        YourSampler::rand()
        {
            return this->randDistribution(this->randEngine);
        }

        ::std::normal_distribution< ::rl::math::Real>::result_type
        YourSampler::gauss()
        {
            return this->gaussDistribution(this->gaussEngine);
        }

        void
        YourSampler::seed(const ::std::mt19937::result_type& value)
        {
            this->randEngine.seed(value);
            this->gaussEngine.seed(value);
        }
    }
}

