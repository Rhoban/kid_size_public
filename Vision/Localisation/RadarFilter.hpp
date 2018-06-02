

#include <mutex>
#include <deque>
#include <string>
#include <RhIO.hpp>

namespace Vision
{
namespace Localisation
{

template<typename T>
class RadarFilter
{
public:
	struct Candidate
	{
    T object;
    float score;
	};

  RadarFilter()
    : bind(NULL)
    {
    }

	virtual ~RadarFilter()
    {
	    if (bind != NULL) {
        delete bind;
        bind = NULL;
	    }
    }

  virtual bool defaultNormalizeSum()
    {
      return true;
    }

  /**
   * Are two objects similar ?
   */
	virtual bool isSimilar(const T &obj1, const T &obj2)=0;

	/**
	 * Do I see the object ?
	 */
	virtual bool isVisible(const T &obj)
    {
      return true;
    }

  /**
   * Converts an object to string
   */
  virtual std::string toString(const T &obj)
    {
      return "?";
    }

	/**
	 * Binds the current filter to RhIO using the given node name
	 */
	virtual bool bindToRhIO(std::string node, std::string command="")
    {
	    if (bind == NULL) {
        bind = new RhIO::Bind(node);

        bind->bindNew("initialScore", initialScore, RhIO::Bind::PullOnly)
          ->comment("Score of a new item")
          ->defaultValue(0.2)
          ->persisted(true);

        bind->bindNew("maximumScore", maximumScore, RhIO::Bind::PullOnly)
          ->comment("Maximum score of an item")
          ->defaultValue(1.0)
          ->persisted(true);

        bind->bindNew("scoreIncrease", scoreIncrease, RhIO::Bind::PullOnly)
          ->comment("Score increase")
          ->defaultValue(0.2)
          ->persisted(true);

        bind->bindNew("scoreReduction", scoreReduction, RhIO::Bind::PullOnly)
          ->comment("Score reduction at each frame")
          ->defaultValue(0.04)
          ->persisted(true);

        bind->bindNew("scoreReductionOut", scoreReductionOut, RhIO::Bind::PullOnly)
          ->comment("Score reduction at each frame where the object should be in the image ")
          ->defaultValue(0.01)
          ->persisted(true);

        bind->bindNew("positionDiscount", positionDiscount, RhIO::Bind::PullOnly)
          ->comment("new = old * disc + seen * (1 - disc)")
          ->defaultValue(0.7)
          ->minimum(0)
          ->maximum(1)
          ->persisted(true);

        bind->bindNew("normalizeSum", normalizeSum, RhIO::Bind::PullOnly)
          ->comment("If enabled, the sum of all elements can never be greater than 1")
          ->defaultValue(defaultNormalizeSum())
          ->persisted(true);

        if (command != "") {
          bind->bindFunc(command, "Show filter candiadtes", 
                         &RadarFilter<T>::showCandidates, *this);
        }

        return true;
	    }

      return false;
    }

  /**
   * Shows the candidates from the filter
   */
  std::string showCandidates()
    {
      std::stringstream ss;

      mutex.lock();
      if (!candidates.size()) {
        ss << "No candidates";
      } else {
        ss << candidates.size() << " candidates:" << std::endl;
        for (auto &candidate : candidates) {
          ss << "- Score: " << candidate.score << ", Object: " << toString(candidate.object) << std::endl;
        }
      }
      mutex.unlock();

      return ss.str();
    }

  /**
   * Getting best candidate
   */
  virtual Candidate getBest()
    {
      Candidate best;
      best.score = -1;

      mutex.lock();
      for (auto &candidate : candidates) {
        if (candidate.score > best.score) {
          best = candidate;
        }
      }
      mutex.unlock();

      return best;
    }

  /**
   * Getting maximum possible score
   */
  virtual double getMaximumScore()
    {
      return maximumScore;
    }

  /**
   * Resets (clear all te candidates);
   */
  virtual void clear()
    {
      mutex.lock();
      candidates.clear();
      mutex.unlock();
    }

	/**
	 * Decrease the scores of all item
	 */
	virtual void decreaseScores()
    {
	    for (auto &candidate : candidates) {
        if (isVisible(candidate.object)) {
          candidate.score -= scoreReduction;
        } else {
          candidate.score -= scoreReductionOut;
        }
	    }
    }

  /// Return the weighted average of o1 and o2 using their weights
  virtual T mergeObjects(const T & o1, const T & o2, double w1, double w2)
    {
      return (o1 * w1 + o2 * w2) / (w1 + w2);
    }

  /**
   * Increase the scores when an object is similar to a candidate
   */
	virtual void increaseScores(const std::vector<T> &objects)
    {
	    // Increasing candidates score if positions of balls declared
	    // match the existing candidates
	    std::vector<Candidate *> toIncrease;

	    for (auto &object : objects) {
        bool matched = false;
        for (auto &candidate : candidates) {
          if (isSimilar(candidate.object, object)) {
            matched = true;

            // Same candidate -> update object properties
            candidate.object = mergeObjects(candidate.object, object,
                                            positionDiscount, 1-positionDiscount);

            toIncrease.push_back(&candidate);
          }
        }
        if (!matched) {
          Candidate candidate;
          candidate.object = object;
          candidate.score = initialScore;
          candidates.push_back(candidate);
        }
	    }

	    // Increasing candidate scores
	    if (toIncrease.size()) {
        float increaseScore = scoreIncrease / toIncrease.size();
        for (auto candidate : toIncrease) {
          candidate->score += increaseScore;
        }
	    }
    }

  /// Merge two similar candidates with weights depending on their scores
  virtual Candidate mergeCandidates(const Candidate & c1, const Candidate & c2)
    {
      Candidate res;
      res.score = c1.score + c2.score;
      res.object =  mergeObjects(c1.object, c2.object, c1.score, c2.score);
      return res;
    }

	virtual void mergeSimilars()
    {
	    std::vector<Candidate> clusters;

	    // For each candidate, try to insert it to existing clusters, otherwise
	    // create a new cluster
	    for (const Candidate & c : candidates) {
        int cluster_id = -1;
        // cci -> cluster candidate index
        for (size_t cci = 0; cci < clusters.size(); cci++) {
          if (isSimilar(clusters[cci].object, c.object)) {
            cluster_id = cci;
            break;
          }
        }

        if (cluster_id == -1) {
          clusters.push_back(c);
        } else {
          // Merge candidates
          const Candidate & cc = clusters[cluster_id];
          clusters[cluster_id] = mergeCandidates(c,cc);
        }
	    }

	    candidates.clear();
	    for (auto &candidate : clusters) {
        candidates.push_back(candidate);
	    }
    }

	/**
	 * Normalize the score, if normalizeSum is true the sum is
	 * normalized to be 1, else the score are just limited to maximumScore
	 */
	virtual void normalizeScores()
    {
	    // Normalizing
	    if (normalizeSum) {
        float total(0);
        for (auto &candidate : candidates) {
          total += candidate.score;
        }
        if (total > maximumScore) {
          for (auto &candidate : candidates) {
            candidate.score /= total;
          }
        }
	    }
	    else {
        for (auto &candidate : candidates) {
          candidate.score = std::min(maximumScore, candidate.score);
        }
	    }
    }

  /**
   * Cleanup the candidates with negative scores
   */
	virtual void clean()
    {
	    // Removing elements with negative score
	    typename std::deque<Candidate>::iterator it = candidates.begin();

	    while (it != candidates.end()) {
        auto &candidate = *it;
        bool remove = false;

        if (candidate.score < 0) {
          remove = true;
        }

        if (remove) {
          it = candidates.erase(it);
        } else {
          it++;
        }
	    }
    }


	/**
	 * Push new observations
	 */
	virtual void newFrame(const std::vector<T> &objects)
    {
	    if (bind != NULL) bind->pull();
	    mutex.lock();

	    decreaseScores();
	    increaseScores(objects);
	    mergeSimilars();
	    normalizeScores();
	    clean();
            
	    mutex.unlock();
	    if (bind != NULL) bind->push();
    }

	/**
	 * Get the current candidates
	 */
	std::deque<Candidate> getCandidates()
    {
      mutex.lock();
      std::deque<Candidate> tmp = candidates;
      mutex.unlock();

      return tmp;
    }

protected:
	// Mutex for thread safe operations
	std::mutex mutex;

	// RhIO binder
	RhIO::Bind *bind;

	// Inital score at candidate creation
	float initialScore;

	// Maximum score
	float maximumScore;

	// Score increase on each perception
	float scoreIncrease;
	float scoreReduction;
	float scoreReductionOut;

	// Discount on merge
	float positionDiscount;

  // Option
  bool normalizeSum;

	std::deque<Candidate> candidates;
};

}
}
