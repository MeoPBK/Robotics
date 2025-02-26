#include "localization/ParticleFilter.h"
#include "localization/Util.h"

#include "tf/tf.h"

using namespace std;

ParticleFilter::ParticleFilter(int numberOfParticles) {
	this->numberOfParticles = numberOfParticles;

	// initialize particles
	for (int i = 0; i < numberOfParticles; i++) {
		this->particleSet.push_back(new Particle());
	}

	// this variable holds the estimated robot pose
	this->bestHypothesis = new Particle();

	// at each correction step of the filter only the laserSkip-th beam of a scan should be integrated
	this->laserSkip = 5;

	// distance map used for computing the likelihood field
	this->distMap = NULL;
}

ParticleFilter::~ParticleFilter() {
	// delete particles
	for (int i = 0; i < numberOfParticles; i++) {
		Particle* p = this->particleSet[i];
		delete p;
	}

	this->particleSet.clear();

	if (this->likelihoodField)
		delete[] this->likelihoodField;

	delete this->bestHypothesis;

	if (this->distMap)
		delete[] this->distMap;
}

int ParticleFilter::getNumberOfParticles() {
	return this->numberOfParticles;
}

std::vector<Particle*>* ParticleFilter::getParticleSet() {
	return &(this->particleSet);
}

void ParticleFilter::initParticlesUniform() {

    //get map properties
    int mapWidth, mapHeight;
    double mapResolution;
    this->getLikelihoodField(mapWidth, mapHeight, mapResolution);
    for (const auto& particle : particleSet) {
        particle->x = Util::uniformRandom(0, mapWidth) * mapResolution;
        particle->y = Util::uniformRandom(0, mapHeight) * mapResolution;
        particle->theta = Util::uniformRandom(-M_PI, M_PI);
        particle->weight = 1.0/double(numberOfParticles);
    }
}

void ParticleFilter::initParticlesGaussian(double mean_x, double mean_y,
		double mean_theta, double std_xx, double std_yy, double std_tt) {
    int mapWidth, mapHeight;
    double mapResolution;
    this->getLikelihoodField(mapWidth, mapHeight, mapResolution);
    for (const auto& particle : particleSet) {
        particle->x = Util::gaussianRandom(mean_x, std_xx);
        particle->y = Util::gaussianRandom(mean_y, std_yy);
        particle->theta = Util::normalizeTheta(Util::uniformRandom(mean_theta, std_tt));
        particle->weight = 1.0/double(numberOfParticles);
    }
}

/**
 *  Initializes the likelihood field as our sensor model.
 */
void ParticleFilter::setMeasurementModelLikelihoodField(
		const nav_msgs::OccupancyGrid& map, double zRand, double sigmaHit) {
	ROS_INFO("Creating likelihood field for laser range finder...");

	// create the likelihood field - with the same discretization as the occupancy grid map
	this->likelihoodField = new double[map.info.height * map.info.width];
	this->likelihoodFieldWidth = map.info.width;
	this->likelihoodFieldHeight = map.info.height;
	this->likelihoodFieldResolution = map.info.resolution;

    // calculates the distance map and stores it in member variable 'distMap'
	// for every map position it contains the distance to the nearest occupied cell.
	calculateDistanceMap(map);

	const double sigmaHitScaled = sigmaHit / likelihoodFieldResolution;
	ROS_INFO_STREAM("sigmaHit: " << sigmaHit << " sigmaHitScaled: " << sigmaHitScaled);
    for (int x = 0; x < likelihoodFieldWidth; x++) {
        for (int y = 0; y < likelihoodFieldHeight; y++) {
          const int coords = x + y * likelihoodFieldWidth;
          const double pHit = Util::gaussian(distMap[coords], sigmaHitScaled, 0.0);
          const double zHit = 1.0 - zRand;
          likelihoodField[coords] = log(zHit * pHit + zRand);
        }
    }
	
	ROS_INFO("...DONE creating likelihood field!");
}

void ParticleFilter::calculateDistanceMap(const nav_msgs::OccupancyGrid& map) {
	// calculate distance map = distance to nearest occupied cell
	distMap = new double[likelihoodFieldWidth * likelihoodFieldHeight];
	int occupiedCellProbability = 90;
	// initialize with max distances
	for (int x = 0; x < likelihoodFieldWidth; x++) {
		for (int y = 0; y < likelihoodFieldHeight; y++) {
			distMap[x + y * likelihoodFieldWidth] = 32000.0;
		}
	}
	// set occupied cells next to unoccupied space to zero
	for (int x = 0; x < map.info.width; x++) {
		for (int y = 0; y < map.info.height; y++) {
			if (map.data[x + y * map.info.width] >= occupiedCellProbability) {
				bool border = false;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						if (!border && x + i >= 0 && y + j >= 0 && x + i
								< likelihoodFieldWidth && y + j
								< likelihoodFieldHeight && (i != 0 || j != 0)) {
							if (map.data[x + i + (y + j) * likelihoodFieldWidth]
									< occupiedCellProbability && map.data[x + i
									+ (y + j) * likelihoodFieldWidth] >= 0)
								border = true;
						}
						if (border)
							distMap[x + i + (y + j) * likelihoodFieldWidth]
									= 0.0;
					}
				}
			}
		}
	}
	// first pass -> SOUTHEAST
	for (int x = 0; x < likelihoodFieldWidth; x++)
		for (int y = 0; y < likelihoodFieldHeight; y++)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}

	// second pass -> NORTHWEST
	for (int x = likelihoodFieldWidth - 1; x >= 0; x--)
		for (int y = likelihoodFieldHeight - 1; y >= 0; y--)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}
}

double* ParticleFilter::getLikelihoodField(int& width, int& height,
		double& resolution) {
	width = this->likelihoodFieldWidth;
	height = this->likelihoodFieldHeight;
	resolution = this->likelihoodFieldResolution;

	return this->likelihoodField;
}

/**
 *  A generic measurement integration method that invokes some specific observation model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::measurementModel(
		const sensor_msgs::LaserScanConstPtr& laserScan) {
	likelihoodFieldRangeFinderModel(laserScan);
}

/**
 *  Method that implements the endpoint model for range finders.
 *  It uses a precomputed likelihood field to weigh the particles according to the scan and the map.
 */
void ParticleFilter::likelihoodFieldRangeFinderModel(
		const sensor_msgs::LaserScanConstPtr & laserScan) {
    for (const auto& particle : particleSet) {
        float angle = laserScan->angle_min;
        int i = 0;
        double q = 0;
        while (angle < laserScan->angle_max) {
            const float r = laserScan->ranges[i];
            if (!isnan(r) && r < laserScan->range_max && r >= laserScan->range_min) {
                const double beam_x_laser = r * cos(angle);
                const double beam_y_laser = r * sin(angle);
                const double beam_x = cos(particle->theta) * beam_x_laser - sin(particle->theta) * beam_y_laser + particle->x;
                const double beam_y = sin(particle->theta) * beam_x_laser + cos(particle->theta) * beam_y_laser + particle->y;
                const int i_value = beam_x / likelihoodFieldResolution;
                const int j_value = beam_y / likelihoodFieldResolution;
                if (i_value >= 0 && j_value >= 0 && i_value < likelihoodFieldWidth && j_value < likelihoodFieldHeight) {
                    // If inside the map
                    q = q + likelihoodField[i_value + j_value * likelihoodFieldWidth];
                }
                else {
                    // Punishment for a beam outside the map
                    q = q + log(1e-2);
                }
            }
            i += laserSkip;
            angle += laserSkip * laserScan->angle_increment;
        }
        particle->weight = exp(q);
    }
}

void ParticleFilter::setMotionModelOdometry(double alpha1, double alpha2,
		double alpha3, double alpha4) {
	this->odomAlpha1 = alpha1;
	this->odomAlpha2 = alpha2;
	this->odomAlpha3 = alpha3;
	this->odomAlpha4 = alpha4;

}

/**
 *  A generic motion integration method that invokes some specific motion model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::sampleMotionModel(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	sampleMotionModelOdometry(oldX, oldY, oldTheta, newX, newY, newTheta);
}

/**
 *  Method that implements the odometry-based motion model.
 */
void ParticleFilter::sampleMotionModelOdometry(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	const double dRot1 = Util::normalizeTheta(atan2(newY - oldY, newX - oldX) - oldTheta);
	const double dTrans = sqrt(pow((oldX - newX), 2.0) + pow((oldY - newY), 2.0));
	const double dRot2 = Util::normalizeTheta(newTheta - oldTheta - dRot1);

    for (const auto& particle : particleSet) {
        // Probabilistic robotics, page 110 - calculate the noisy movement
        const double dRot1New = dRot1 - Util::gaussianRandom(0.0, odomAlpha1*dRot1 + odomAlpha2*dTrans);
        const double dTransNew = dTrans - Util::gaussianRandom(0.0, odomAlpha3*dTrans + odomAlpha4*dRot2);
        const double dRot2New = dRot2 - Util::gaussianRandom(0.0, odomAlpha1*dRot2 + odomAlpha2*dTrans);
        const double old_x = particle->x;
        const double old_y = particle->y;
        const double old_theta = particle->theta;
        particle->x = particle->x + dTransNew*cos(particle->theta + dRot1New);
        particle->y = particle->y + dTransNew*sin(particle->theta + dRot1New);
        particle->theta = Util::normalizeTheta(particle->theta + dRot1New + dRot2New);

        // Probabilistic robotics, page 108 - modify the motion probability
        const double dRot1New_prob = Util::normalizeTheta(atan2(particle->y - old_y, particle->x - old_x) - old_theta);
        const double dTransNew_prob = sqrt(pow((old_x - particle->x), 2.0) + pow((old_y - particle->y), 2.0));
        const double dRot2New_prob = Util::normalizeTheta(particle->theta - old_theta - dRot1New_prob);
        const double normFactor = Util::gaussian(0.0, 1.0, 0.0);
        const double p1 = Util::gaussian(Util::normalizeTheta(dRot1 - dRot1New_prob),  odomAlpha1*dRot1New_prob + odomAlpha2*dTransNew_prob, 0.0) / normFactor;
        const double p2 = Util::gaussian(dTrans - dTransNew_prob,  odomAlpha3*dTransNew_prob + odomAlpha4*(Util::normalizeTheta(dRot1New_prob+dRot2New_prob)), 0.0) / normFactor;
        const double p3 = Util::gaussian(Util::normalizeTheta(dRot2 - dRot2New_prob),  odomAlpha1*dRot2New_prob + odomAlpha2*dTransNew_prob, 0.0) / normFactor;
        particle->weight *= p1*p2*p3;
    }
}

/**
 *  The stochastic importance resampling.
 */
void ParticleFilter::resample() {
    vector<Particle> newParticleSet {};
    double cdf[numberOfParticles];
    int i = 0;
	for (const auto& particle: particleSet) {
        if (i == 0) {
            cdf[0] = particle->weight;
            i++;
            continue;
        }
        cdf[i] = cdf[i-1] + particle->weight;
        i++;
	}
	// Normalize the CDF and particle weights
	for (i = 0; i < numberOfParticles; i++) {
	    cdf[i] /= cdf[numberOfParticles-1];
        particleSet[i]->weight /= cdf[numberOfParticles-1];
	}
	i = 0;
	double u[numberOfParticles] = {0.0};
	u[0] = Util::uniformRandom(0, 1.0/double(numberOfParticles));
	for (int j = 0; j < numberOfParticles; j++) {
        while (u[j] > cdf[i]) {
            if(i >= numberOfParticles-1) {
                break;
            }
            i++;
        }
        newParticleSet.push_back(Particle(particleSet[i]));
        u[j+1] = u[j] + 1.0/double(numberOfParticles);
	}
    for (int i = 0; i < numberOfParticles; i++) {
        particleSet[i]->x = newParticleSet[i].x;
        particleSet[i]->y = newParticleSet[i].y;
        particleSet[i]->theta = newParticleSet[i].theta;
        particleSet[i]->weight = 1.0/double(numberOfParticles);
    }
}

Particle* ParticleFilter::getBestHypothesis() {
    double max_weight = particleSet[0]->weight;
    Particle* best_particle_found = particleSet[0];
    for (const auto& particle : particleSet) {
        if (particle->weight > max_weight) {
            max_weight = particle->weight;
            best_particle_found = particle;
        }
    }
    bestHypothesis = best_particle_found;
	return this->bestHypothesis;
}

// added for convenience
int ParticleFilter::computeMapIndex(int width, int height, int x,
		int y) {
	return x + y * width;
}

