//
//  ConfigurablePath.h
//  abstract class to describe configurable path like ellipse, line, kcurve, .etc
//  Created by Ning Liu on 3/27/16.
//

#ifndef ConfigurablePath_h
#define ConfigurablePath_h

#include <vector>
#include <cmath>
#include <iostream>
#include <assert.h>

namespace PredictedPath
{

struct Sample
{
    Sample(){}
    Sample(float ix, float iy):x(ix),y(iy){}
    Sample(float ix, float iy, const std::vector<float>& idata):x(ix), y(iy), metaData(idata){}
    // Sample(float ix, float iy,std::vector<float> imetaData):x(ix), y(iy),metaData(imetaData){}
    float x, y;
    bool bound=false;
    std::vector<float> metaData;
    double distance(const Sample &s){
        return sqrt(pow(x-s.x, 2)+pow(y-s.y, 2));
    }
	Sample& operator=(const Sample& s){
		this->x=s.x;
		this->y=s.y;
		return *this;
	}
};

typedef std::vector<Sample> Samples;

const double LARGE_DOUBLE =  std::numeric_limits<double>::max();

struct FloatPoint{
    FloatPoint(float ix, float iy) : x(ix), y(iy){}
    float x;
    float y;
};

/// the manipulate point position
struct Pivot{
    Pivot(float ix, int iy, bool isel = false) : x(ix), y(iy), isSelected(isel) {}
    void set(float ix, int iy, bool isel)
    {
        x = ix;
        y = iy;
        isSelected = isel;
    }
    
    float x, y;
    bool isSelected;
};

/// 2d transform class
class AffineTransform2d{
public:
    AffineTransform2d():a00(1.0f), a10(0), a01(0), a11(0), b0(1.0f), b1(0){}
    
    AffineTransform2d(float A00, float A10, float A01, float A11, float B0, float B1):
    a00(A00), a10(A10), a01(A01), a11(A11), b0(B0), b1(B1){}
    
    AffineTransform2d(const AffineTransform2d& xform) noexcept
    {
        a00 = xform.a00; a10 = xform.a10; b0 = xform.b0;
        a01 = xform.a01; a11 = xform.a11; b1 = xform.b1;
    }
    
    AffineTransform2d& operator= (const AffineTransform2d& xform) noexcept
    {
        a00 = xform.a00; a10 = xform.a10; b0 = xform.b0;
        a01 = xform.a01; a11 = xform.a11; b1 = xform.b1;
        return *this;
    }
    
    ~AffineTransform2d(){}
    
    FloatPoint transformPt(float x, float y) noexcept
    {
        return FloatPoint( a00 * x + a10 * y + b0, a01 * x + a11 * y + b1 );
    }
    
    bool isIdentity() noexcept
    {
        return  (a00 == 1.0f)    && (a10 == 0)      && (b0 == 0) &&
                (a10 == 0)       && (a11 == 1.0f)   && (b1 == 0);
    }
    
private:
    /* The transform matrix is: A * x + b
     (a00 a10 b0)
     (a01 a11 b1)
     ( 0   0   1)
     */
    float a00, a10, a01, a11, b0, b1;
};

class ConfigurablePath{
public:
    /// construct path from input discrete samples
    ConfigurablePath (const Samples& inputSamples, float segSize)
    {
        // copy to inner samples, other operations will be handled by sub-classes
//        for(int i=0; i<inutSamples.size(); i++)
//            m_samples.push_back(inputSamples[i]);
        
        // insert more points between sparse segments backward
        int curIdx = 0;
        int nextIdx = curIdx + 1;
        // push the first element
        m_samples.push_back(inputSamples[0]);
        
        // meature the next point to current point
        while( nextIdx < inputSamples.size())
        {
            float cx = inputSamples[curIdx].x;
            float cy = inputSamples[curIdx].y;
        
            float nx = inputSamples[nextIdx].x;
            float ny = inputSamples[nextIdx].y;
            
            float dis = sqrtf((cx-nx)*(cx-nx) + (cy-ny)*(cy-ny));
            
            if(dis > segSize)
//			if(true)
            {
                // insert more points between cur and next
                int nNewPoints = dis / segSize;
                
                for(int k = 1; k <= nNewPoints; k++)
                {
                    float t = (float)k / (float)(nNewPoints+1);
                    float npx = lerp(t, cx, nx);
                    float npy = lerp(t, cy, ny);
                    
                    // resample the meta data
                    std::vector<float> metaData;
                    const std::vector<float> &cd = inputSamples[curIdx].metaData;
                    const std::vector<float> &nd = inputSamples[nextIdx].metaData;
                    
                    assert(cd.size() == nd.size());
                    
                    for(int i = 0; i < cd.size(); i++)
                        metaData.push_back(lerp(t, cd[i], nd[i]));
                    
                    m_samples.push_back(Sample(npx, npy, metaData));
                }
                
                //printf("insert %d point\n", nNewPoints);
                
                // be sure to push next itself
                Sample *temp=const_cast<Sample*>(&inputSamples[nextIdx]);
                temp->bound=true;
                m_samples.push_back(inputSamples[nextIdx]);
                
                // move to next point
                curIdx = nextIdx;
                nextIdx++;

            }
            else
            {
                // skip next point
                nextIdx++;
                
                //printf("skip one point\n");
            }
        }
        
    }

    virtual ~ConfigurablePath ()
    {
        m_samples.clear();
        m_pivots.clear();
    }
    
    /// set to a new pivot position, also indicates it is being selected and new samples are calculated
    void setPivot(int i, float x, float y)
    {
        if( i >= 0 && i <= m_pivots.size() )
            m_pivots[i].set(x, y, true);
        
        // re-calculate samples
        projectSamples();
    }
    
    /// set current transform, will not trigger re-calculation
    void setTransform(AffineTransform2d xform)
    {
        m_transform = xform;
    }
    
    /// get how many sample point in this shape
    size_t getSampleNumber() { return m_samples.size();}
    
    /// get new sample
    Sample getSample(int i)
    {
        Sample s = m_samples[i];
        if(!m_transform.isIdentity())
        {
            FloatPoint p = m_transform.transformPt(s.x, s.y);
            s.x = p.x;
            s.y = p.y;
        }
        
        return s;
    }
    
    /// get the stroke without transform
    Samples getRawSamples() const {return m_samples;}
    
    /// get fitting error
    double getFittingError(){return m_error;}
    
    /// fit the init shape
    virtual double fitShape() = 0;
    
    /// allocate pivots for this SHAPE(called only once), you should call project inside it
    virtual void buildPivots() = 0;
    
    /// update current samples according to pivots and transform
    virtual double projectSamples() = 0;
    
    /// hit test
    virtual bool isOnPath(float x, float y) = 0;
    
    /// get the nearest pivot to manipulate 1s4
    virtual int getNearestPivot(float x, float y) = 0;

private:
    // linear interpolate
    float lerp(float t, float minVal, float maxVal) noexcept {return (1.0f-t) * minVal + t * maxVal;}
    
protected:
    /// holds the samples generated
    std::vector<Sample>  m_samples;
    std::vector<Pivot>   m_pivots;
    AffineTransform2d    m_transform;
    double               m_error;
};

class Kcurve : public ConfigurablePath{
public:
    Kcurve(const Samples& inputSamples, float segSize = 10.0f, float smoothness = 0.2f);
    
    double fitShape() override;
    void buildPivots() override;
    double projectSamples() override;
    bool isOnPath(float x, float y) override;
    int getNearestPivot(float x, float y) override;
    
private:
    double smooth_weight; //[0.1, 50]
    double length_weight = 1.0f;

};
    
} // end namespace



#endif /* ConfigurablePath_h */
