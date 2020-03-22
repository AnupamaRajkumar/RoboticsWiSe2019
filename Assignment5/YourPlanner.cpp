#include "YourPlanner.h"
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"
YourPlanner::YourPlanner() :
  RrtConConBase()
{
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "Your Planner";
}

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
	RrtConConBase::choose(chosen);
}


RrtConConBase::Neighbor
YourPlanner::nearest(const Tree &tree, const ::rl::math::Vector &chosen)
{
  //create an empty pair <Vertex, distance> to return
  Neighbor p(Vertex(), (::std::numeric_limits<::rl::math::Real>::max)());
  int numVer(::boost::num_vertices(tree));
  int rangeN(200);
  int j(0);
  //Iterate through all vertices to find the nearest neighbour
  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {
      if(j > numVer - rangeN){
          ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q) ;
          if (d < p.second)
          {
              p.first = *i.first;
              p.second = d;

          }
      }
      j++;
  }

  // Compute the square root of distance
  p.second = this->model->inverseOfTransformedDistance(p.second);

  return p;
}




RrtConConBase::Vertex 
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
    //Do first extend step

  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = distance;

  bool reached = false;

  if (step <= this->delta)
  {
    reached = true;
  }
  else
  {
    step = this->delta;
  }

  ::rl::plan::VectorPtr last = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

  // move "last" along the line q<->chosen by distance "step / distance"
  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);

  this->model->setPosition(*last);
  this->model->updateFrames();

  if (this->model->isColliding())
  {
    //tree[nearest.first].fails += 1;
    return NULL;
  }

  ::rl::math::Vector next(this->model->getDof());

  while (!reached)
  {
    //Do further extend step

    distance = this->model->distance(*last, chosen);
    step = distance;

    if (step <= this->delta)
    {
      reached = true;
    }
    else
    {
      step = this->delta;
    }

    // move "next" along the line last<->chosen by distance "step / distance"
    this->model->interpolate(*last, chosen, step / distance, next);

    this->model->setPosition(next);
    this->model->updateFrames();

    if (this->model->isColliding())
    {
      break;
    }

    *last = next;
  }

Vertex connected = this->addVertex(tree, last);
this->addEdge(nearest.first, connected, tree);
//tree[connected].fails = 0;

return connected;
}

RrtConConBase::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
    ::rl::math::Real distance = nearest.second;
    ::rl::math::Real step = (::std::min)(distance, this->delta);

    ::rl::plan::VectorPtr next = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

    //if the nodes are not colliding interpolate by taking bigger steps else take smaller steps

    if(!this->model->isColliding())
    {
        this->model->interpolate(*tree[nearest.first].q, chosen, distance, *next);
    }
    else
    {
        this->model->interpolate(*tree[nearest.first].q, chosen, step, *next);
    }

    this->model->setPosition(*next);
    this->model->updateFrames();

    if (!this->model->isColliding())
    {
        Vertex extended = this->addVertex(tree, next);
        this->addEdge(nearest.first, extended, tree);
        return extended;
    }

    //tree[nearest.first].fails +=1;
    return NULL;
}

bool
YourPlanner::solve()
{
    this->time = ::std::chrono::steady_clock::now();
    // Define the roots of both trees
    this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector >(*this->start));
    this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared< ::rl::math::Vector >(*this->goal));

    Tree* a = &this->tree[0];
    Tree* b = &this->tree[1];

    this->tree[0][this->begin[0]].fails = 0;
    this->tree[1][this->begin[1]].fails = 0;

    ::rl::math::Vector chosen(this->model->getDof());

    while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
    {
        //First grow tree a and then try to connect b.
        //then swap roles: first grow tree b and connect to a.
        //start and goal based gaussian sampling
        for (::std::size_t j = 0; j < 2; ++j)
        {
            ::rl::math::Vector dof(this->model->getDof());
            if(j==0){
                dof = *this->start;
                //dof = this->sampler->generate();
                for(int i = 0; i < chosen.size();i++){
                    chosen[i] = YourPlanner::gaussianRandom(dof[i], 2.0);
                }
            }else{
                dof = *this->goal;
                //dof = this->sampler->generate();
                for(int i = 0; i < chosen.size();i++){
                    chosen[i] = YourPlanner::gaussianRandom(dof[i], 2.0);
                }
            }

            //Find the nearest neighbour in the tree
            Neighbor aNearest = this->nearest(*a, chosen);

            //Do a EXTEND step from the nearest neighbour to the sample
            Vertex aExtended = this->extend(*a, aNearest, chosen);

            //If a new node was inserted tree a
            if (NULL != aExtended)
            {
                // Try a CONNECT step form the other tree to the sample
                Neighbor bNearest = this->nearest(*b, *(*a)[aExtended].q);
                Vertex bConnected = this->connect(*b, bNearest, *(*a)[aExtended].q);

                if (NULL != bConnected)
                {
                    //Test if we could connect both trees with each other
                    if (this->areEqual(*(*a)[aExtended].q, *(*b)[bConnected].q))
                    {
                        this->end[0] = &this->tree[0] == a ? aExtended : bConnected;
                        this->end[1] = &this->tree[1] == b ? bConnected: aExtended;
                        return true;
                    }
                }
            }

            //Swap the roles of a and b
            using ::std::swap;
            swap(a, b);
        }
    }
    return false;
}
//Implementation reused from Util.cpp used in Assignment 4
//---------------------------------------------------
double
YourPlanner::gaussianRandom( double mean, double std ) {
	const double norm = 1.0 / (RAND_MAX + 1.0);
	double u = 1.0 - rand()*norm;
	double v = rand()*norm;
	double z = sqrt(-2.0*log(u))* cos(2.0*M_PI*v);
	return mean + std*z;
}



