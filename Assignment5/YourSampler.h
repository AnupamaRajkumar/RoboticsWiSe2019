#ifndef _YOURSAMPLER_H_
#define _YOURSAMPLER_H_


//#include <rl/plan/Sampler.h>
#include <rl/plan/GaussianSampler.h>
#include <rl/plan/UniformSampler.h>
#include <random>

namespace rl
{
    namespace plan
    {
        /**
         * Uniform random sampling strategy.
         */
        class YourSampler : public Sampler
        {
        public:
            YourSampler();

            virtual ~YourSampler();

            ::rl::math::Vector generate();

            ::rl::math::Vector generateCollisionFree();

            virtual void seed(const ::std::mt19937::result_type& value);

            ::std::normal_distribution< ::rl::math::Real>::result_type gauss();

            ::std::normal_distribution< ::rl::math::Real> gaussDistribution;

            ::std::mt19937 gaussEngine;

            //::rl::math::Vector* sigma;

        protected:
            ::std::uniform_real_distribution< ::rl::math::Real>::result_type rand();

            ::std::uniform_real_distribution< ::rl::math::Real> randDistribution;

            //::std::normal_distribution< ::rl::math::Real>::result_type gauss();

            //::std::normal_distribution< ::rl::math::Real> gaussDistribution;

            ::std::mt19937 randEngine;

            //::std::mt19937 gaussEngine;

        private:

        };
    }
}


#endif // _YOURSAMPLER_H_
