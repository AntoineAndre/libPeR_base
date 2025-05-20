/*!
 \file prRegularlySampledCSImageDepth.h
 \brief Header file for the prRegularlySampledCSImageDepth class
 \author Guillaume CARON
 \version 0.1
 \date december 2021
 */

#if !defined(_PRREGULARLYSAMPLEDCSIMAGEDEPTH_H)
#define _PRREGULARLYSAMPLEDCSIMAGEDEPTH_H

#include <per/prRegularlySampledCSImage.h>
#include <per/prCartesian3DPointVec.h>

#include <per/prStereoModel.h>
#include <per/prOmni.h>
#include <per/prEquirectangular.h>
#include <per/prPointFeature.h>

//#include <per/prcommon.h>

#include <visp/vpImage.h>

/*!
 \class PER_EXPORT prRegularlySampledCSImage prRegularlySampledCSImage.h <per/prRegularlySampledCSImage.h>
 \brief Class defining the image place as a set of spherical points in Cartesian coordinates lying 
 *      on a unit sphere and regularly sampled, in the geodesic meaning
 */
//template<typename TImage, typename TDepth> class PER_EXPORT prRegularlySampledCSImageDepth : public prRegularlySampledCSImage<TImage> //double templates creates conflicts with single template expecting classes --> TDepth = float before finding a better solution
typedef float TDepth; // temporary trick 
template<typename TImage> class PER_EXPORT prRegularlySampledCSImageDepth : public prRegularlySampledCSImage<TImage>
{
public:
    
    /*!
     * \fn prRegularlySampledCSImageDepth(unsigned int _subdivLevels = 3)
     * \brief Constructor of a prRegularlySampledCSImageDepth object (CS stands for Cartesian Spherical) : initializes the number of subdivision level of the sphere, i.e. the resolution of the spherical image / depth map, and loads the spherical points.
     */
    prRegularlySampledCSImageDepth(unsigned int _subdivLevels = 3) : prRegularlySampledCSImage<TImage>::prRegularlySampledCSImage(_subdivLevels), bitmapDepth(NULL), isGe3DSpaceSet(false)
    {
    	std::cout << "nbSamples: " << nbSamples <<std::endl;
			//Allocate the depthmap of the same size as the image
			bitmapDepth = new TDepth[nbSamples];
			TDepth *pt_bitmapDepth = bitmapDepth;
			for(unsigned long ns = 0 ; ns < nbSamples ; ns++, pt_bitmapDepth++)
      {            
        *pt_bitmapDepth = 1.f;
			}
			
			std::cout << "(*bitmapDepth): " << (*bitmapDepth) << std::endl;

			//Allocate the 3D point cloud that will be computed from spherical coordinates and provided depth
			ge3DSpace = new prCartesian3DPointVec[nbSamples];

			prCartesian3DPointVec *pt_XS = (prCartesian3DPointVec *)ge;
			prCartesian3DPointVec *pt_X = (prCartesian3DPointVec *)ge3DSpace;

      for(unsigned long ns = 0 ; ns < nbSamples ; ns++, pt_XS++, pt_X++)
      {            
        //*pt_X = *pt_XS;
        pt_X->setPoint(pt_XS->get_X(),pt_XS->get_Y(),pt_XS->get_Z());
			}
			isGe3DSpaceSet = true;
    }

    /*!
     * \fn ~prRegularlySampledCSImageDepth()
     * \brief Destructor of the prRegularlySampledCSImageDepth object : frees memory of depthmap values (the rest is done by the parent class).
     */
    virtual ~prRegularlySampledCSImageDepth() override
    {
        if(bitmapDepth != NULL)
				{
            delete [] bitmapDepth;
						bitmapDepth = NULL;
				}

        if(ge3DSpace != NULL)
				{
            delete [] ge3DSpace;
						ge3DSpace = NULL;
				} 				
    }


		using prRegularlySampledCSImage<TImage>::buildFromEquiRect;

		/*!
     * \fn int buildFromEquiRect(vpImage<TImage> & I, vpImage<TDepth> & depthmap, prEquirectangular & equiCam, vpImage<unsigned char> *Mask = NULL, vpHomogeneousMatrix *cMi = NULL)
     * \brief Builds the pixels map of the spherical image from the acquired equirectangular image data
     * TODO: to generalize for other sensor types than the equirectangular by merging with buildFromTwinOmni
     * \param I the input image of which pixels are of type T
		 * \param depthmap the input image of which pixels are of type float (move to template to allow double or else?)
     * \param equiCam the dual omni camera intrinsic and extrinsic parameters
     * \param Mask the pointer to a Mask image in order to ignore some pixels (if NULL, every pixel of the input image is considered)
     * \param cMi the pointer to a transformation matrix from frame F_i (usually initial) to frame F_c (usually current) (if NULL, no transformation is considered)
     * \return  0 if the spherical image is well built
     *         -1 if there is no spherical image points loaded
     */
    int buildFromEquiRect(vpImage<TImage> & I, vpImage<TDepth> & depthmap, prEquirectangular & equiCam, vpImage<unsigned char> *Mask = NULL, vpHomogeneousMatrix *cMi = NULL)
    {
        if(nbSamples == 0)
            return -1;
            
            if(cMi != NULL)
            std::cout << *cMi << std::endl;
        
        if(inttyp == IMAGEPLANE_BILINEAR)
        {
            if(bitmapf != NULL)
            {
                delete [] bitmapf;
                bitmapf = NULL;
            }
            bitmapf = new float[nbSamples];
            isBitmapfSet = true;
        }
        
        TImage *pt_bitmap = bitmap;
        float *pt_bitmapf = bitmapf;
				TDepth *pt_bitmapDepth = bitmapDepth;

        prCartesian3DPointVec XSs;
        prCartesian3DPointVec *pt_XS = (prCartesian3DPointVec *)ge;
				prCartesian3DPointVec *pt_X = (prCartesian3DPointVec *)ge3DSpace;
        unsigned int icam = 0, imWidth = I.getWidth(), imHeight = I.getHeight(), i, j;
        prPointFeature P;
        double u, v, du, dv, unmdu, unmdv;

        for(unsigned long ns = 0 ; ns < nbSamples ; ns++, pt_XS++, pt_X++, pt_bitmap++, pt_bitmapDepth++)
        {
            //bool consider = false;
            *pt_bitmap = 0;
            if(inttyp == IMAGEPLANE_BILINEAR)
                *pt_bitmapf = 0;
						*pt_bitmapDepth = 0;
                        
            P.sX = *pt_XS;
            
            equiCam.project3DImage(P);
            
            equiCam.meterPixelConversion(P);
            
            u = P.get_u();
            v = P.get_v();
                
            if( (u >= 0) && (v >= 0) && (u < (imWidth-1)) && (v < (imHeight-1))  )
            {
                switch(inttyp)
                {
                    case IMAGEPLANE_BILINEAR:
                        i = (int)v; dv = v-i; unmdv = 1.0-dv;
                        j = (int)u; du = u-j; unmdu = 1.0-du;
                        
                        if((Mask != NULL))
                        {
                            if ((*Mask)[i][j] != 0)
                            {
                                *pt_bitmapf += I[i][j]*unmdv*unmdu;
																*pt_bitmapDepth += depthmap[i][j]*unmdv*unmdu;
                                //consider = true;
                            }

                            if ((*Mask)[i+1][j] != 0)
                            {
                                *pt_bitmapf += I[i+1][j]*dv*unmdu;
																*pt_bitmapDepth += depthmap[i+1][j]*dv*unmdu;
                                //consider = true;
                            }

                            if ((*Mask)[i][j+1] != 0)
                            {
                                *pt_bitmapf += I[i][j+1]*unmdv*du;
																*pt_bitmapDepth += depthmap[i][j+1]*unmdv*du;
                                //consider = true;
                            }

                            if ((*Mask)[i+1][j+1] != 0)
                            {
                                *pt_bitmapf += I[i+1][j+1]*dv*du;
																*pt_bitmapDepth += depthmap[i+1][j+1]*dv*du;
                                //consider = true;
                            }
                        }
                        else
                        {
                            *pt_bitmapf = I[i][j]*unmdv*unmdu + I[i+1][j]*dv*unmdu + I[i][j+1]*unmdv*du + I[i+1][j+1]*dv*du;
														*pt_bitmapDepth = depthmap[i][j]*unmdv*unmdu + depthmap[i+1][j]*dv*unmdu + depthmap[i][j+1]*unmdv*du + depthmap[i+1][j+1]*dv*du;
                            //consider = true;
                        }
                        
                        *pt_bitmap = *pt_bitmapf;
                        
                        break;
                    case IMAGEPLANE_NEARESTNEIGH:
                    default:
                        
                        i = vpMath::round(v);
                        j = vpMath::round(u);
                        
                        if(Mask != NULL)
                        {
                            if((*Mask)[i][j] != 0)
                            {
                                *pt_bitmap = I[i][j];
																*pt_bitmapDepth = depthmap[i][j];
                                //consider = true;
                            }
                        }
                        else
                        {
                            *pt_bitmap = I[i][j];
														*pt_bitmapDepth = depthmap[i][j];
                            //consider = true;
                        }
                        
                        break;
                }
            }

						//std::cout << "(*pt_bitmapDepth): " << (*pt_bitmapDepth) << " / " << depthmap[i][j] << std::endl;
						if((*pt_bitmapDepth) > 0.f)
						{
							//std::cout << "pt_X->get_X(): " << pt_X->get_X();
							pt_X->setPoint(pt_XS->get_X()*(*pt_bitmapDepth), pt_XS->get_Y()*(*pt_bitmapDepth), pt_XS->get_Z()*(*pt_bitmapDepth)); // or = (*pt_XS) * 
							//std::cout << " / " << pt_X->get_X();
							
							if(cMi != NULL) // very rough! Real rendering and resampling should be
            	{
            		double Xs, Ys, Zs;
            		
            		P.sX = pt_X->changeFrame(*cMi);
            		pt_X->setPoint(P.sX.get_X(), P.sX.get_Y(), P.sX.get_Z());
            		            		
            		equiCam.project3DSphere(P, Xs, Ys, Zs);
            		
            		//std::cout << std::endl << Xs << " "  << Ys << " "  << Zs;
            		//std::cout << " / " << pt_XS->get_X()<< "  " << pt_XS->get_Y()<< "  " << pt_XS->get_Z();
            		
            		pt_XS->setPoint(Xs, Ys, Zs);
            		
            		//std::cout << "(*pt_bitmapDepth): " << (*pt_bitmapDepth);
            		
            		*pt_bitmapDepth = sqrt(pt_X->get_X()*pt_X->get_X() + pt_X->get_Y()*pt_X->get_Y() + pt_X->get_Z()*pt_X->get_Z());
            		
            		//std::cout << " / " << (*pt_bitmapDepth) << std::endl;
            	}
 						}

            if(inttyp == IMAGEPLANE_BILINEAR)
                pt_bitmapf++;
        }
        
        return 0;
    }
    
    /*!
     * \fn int toEquiRect(vpImage<T> & I_r, vpPoseVector & r, prEquirectangular & equiCam, vpImage<unsigned char> *Mask = NULL)
     * \brief Projects the spherical image to an equirectangular image (mainly for visualization purpose)
     * TODO: to generalize for other sensor types than the equirectangular by merging with toTwinOmni
     * \param I_r (output) the output image of which pixels are of type T
     * \param r the pose vector of the frame change to apply to the spherical image before projection in the dual omni image plane
     * \param equiCam the equirectangular camera intrinsic and extrinsic parameters
     * \param Mask the pointer to a Mask image in order to ignore some pixels (if NULL, every pixel of the input image is considered ; if not NULL, masked pixels are not changed)
     * \return  0 if the spherical image is well built
     *         -1 if there is no spherical image points loaded
     */
    int toEquiRect(vpImage<TImage> & I_r, vpPoseVector & r, prEquirectangular & equiCam, vpImage<unsigned char> *Mask = NULL)
    {
        if(nbSamples == 0)
            return -1;
        
        TImage *pt_bitmap = bitmap;
        float *pt_bitmapf = bitmapf;

        prCartesian3DPointVec *pt_X = (prCartesian3DPointVec *)ge3DSpace;
        unsigned int icam = 0, imWidth = I_r.getWidth(), imHeight = I_r.getHeight(), i, j;
        prPointFeature P;
        double u, v, du, dv, unmdu, unmdv;
        
        vpHomogeneousMatrix dMc;
        dMc.buildFrom(r);
        dMc = dMc.inverse(); //a justifier clairement dans la doc
        
        for(unsigned long ns = 0 ; ns < nbSamples ; ns++, pt_X++, pt_bitmap++)
        {
            P.sX = *pt_X;
            
            P.sX = P.sX.changeFrame(dMc);

            equiCam.project3DImage(P);
            
            equiCam.meterPixelConversion(P);
                
            u = P.get_u();
            v = P.get_v();
            
            if( (u >= 0) && (v >= 0) && (u < (imWidth-1)) && (v < (imHeight-1))  )
            {
                i = (unsigned int)v;
                j = (unsigned int)u;
                
                if((Mask != NULL))
                {
                    if ((*Mask)[i][j] != 0)
                       I_r[i][j] = *pt_bitmap;
                }
                else
                {
                    I_r[i][j] = *pt_bitmap;
                }
            }
        }
        
        return 0;
    }

    

    /*!
     * \fn int getRawSample(unsigned long i, prCartesian3DPointVec & pXS, T & val)
     * \brief Extract the sample position and raw value at given index
     * \param i the index at which to extract the raw
     * \param pXS (output) the output coordinates of the sample
     * \param val (output) the output raw value at the sample coordinates
     * \return  0 if the raw sample could be got
     *         -1 if the index is out of range
     */
    int getRawSample(unsigned long i, prCartesian3DPointVec & pXS, TImage & val)
    {
        if(i >= nbSamples)
            return -1;
        
        pXS = ((prCartesian3DPointVec *)ge)[i];
        val = bitmap[i];
        
        return 0;
    }
    
    /*!
     * \fn int getSample(unsigned long i, prCartesian3DPointVec & pXS, T & val)
     * \brief Extract the sample position and value (that might have been interpolated) at given index
     * \param i the index at which to extract the raw
     * \param pXS (output) the output coordinates of the sample
     * \param val (output) the output value at the sample coordinates
     * \return  0 if the sample could be got
     *         -1 if the index is out of range
     */
    int getSample(unsigned long i, prCartesian3DPointVec & pXS, float & val)
    {
        if(i >= nbSamples)
            return -1;
        
        if(!isBitmapfSet)
            return -2;
        
        pXS = ((prCartesian3DPointVec *)ge)[i];
        val = bitmapf[i];
        
        return 0;
    }
    
		
    /*!
     * \fn prRegularlySampledCSImageDepth<TImage, TDepth> &operator=(const prRegularlySampledCSImageDepth<TImage, TDepth> &RSCSID)
     * \brief = operator override to copy prRegularlySampledCSImage object
     * TODO: better deal with the memcpy when pixels are not of a basic type
     * TODO: check if ge[j] = RSCSI.ge[j];  needs pointer casts
     * \param RSCSID the prRegularlySampledCSImageDepth to copy to the current
     * \return the prRegularlySampledCSImage<T> self reference object
     */
    /*prRegularlySampledCSImageDepth<TImage, TDepth> &operator=(const prRegularlySampledCSImageDepth<TImage, TDepth> &RSCSID)
    {
        subdivLevels = RSCSID.subdivLevels;
        inttyp = RSCSID.inttyp;
        nbSamples = RSCSID.nbSamples;
        isBitmapfSet = RSCSID.isBitmapfSet;
        
        bitmap = new TImage[nbSamples];
        std::memcpy(bitmap, RSCSID.bitmap, nbSamples*sizeof(TImage)); // OK si T est un type de base
				bitmapDepth = new TDepth[nbSamples];
				std::memcpy(bitmapDepth, RSCSID.bitmapDepth, nbSamples*sizeof(TDepth)); // OK si T est un type de base	
        ge = new prCartesian3DPointVec[nbSamples];
				ge3DSpace = new prCartesian3DPointVec[nbSamples];
        for(int j = 0 ; j < nbSamples ; j++)
				{
            ge[j] = RSCSID.ge[j];
						ge3DSpace[j] = RSCSID.ge3DSpace[j];  
				}
        if(isBitmapfSet)
        {
            bitmapf = new float[nbSamples];
            std::memcpy(bitmapf, RSCSID.bitmapf, nbSamples*sizeof(float));
        }
        
        return *this;
    }
		*/


    /*!
    * \fn prRegularlySampledCSImageDepth<TImage> &operator=(const prRegularlySampledCSImageDepth<TImage> &RSCSID)
    * \brief = operator override to copy prRegularlySampledCSImage object
    * TODO: better deal with the memcpy when pixels are not of a basic type
    * TODO: check if ge[j] = RSCSI.ge[j];  needs pointer casts
    * \param RSCSID the prRegularlySampledCSImageDepth to copy to the current
    * \return the prRegularlySampledCSImage<T> self reference object
    */
    prRegularlySampledCSImageDepth<TImage> &operator=(const prRegularlySampledCSImageDepth<TImage>& RSCSID)
    {
        subdivLevels = RSCSID.subdivLevels;
        inttyp = RSCSID.inttyp;
        nbSamples = RSCSID.nbSamples;
        isBitmapfSet = RSCSID.isBitmapfSet;
        
        bitmap = new TImage[nbSamples];
        std::memcpy(bitmap, RSCSID.bitmap, nbSamples * sizeof(TImage)); // OK si T est un type de base
		bitmapDepth = new TDepth[nbSamples];
		std::memcpy(bitmapDepth, RSCSID.bitmapDepth, nbSamples * sizeof(TDepth)); // OK si T est un type de base	
        ge = new prCartesian3DPointVec[nbSamples];
		ge3DSpace = new prCartesian3DPointVec[nbSamples];
        for (int j = 0; j < nbSamples; j++)
		{
            ge[j] = RSCSID.ge[j];
			ge3DSpace[j] = RSCSID.ge3DSpace[j];  
		}
		isGe3DSpaceSet = RSCSID.isGe3DSpaceSet;
        if (isBitmapfSet)
        {
            bitmapf = new float[nbSamples];
            std::memcpy(bitmapf, RSCSID.bitmapf, nbSamples * sizeof(float));
        }
        
        return *this;
    }
		

    /*!
     * \fn unsigned int getSampleDim()
     * \brief accessor to the number of dimensions of a point sample
     * TODO: make it more generic
     * \return the number of dimensions of a point sample
     */
    unsigned int getSampleDim()
    {
      return 3;
    }

    /*!
     * \fn int changeSampleFrame(unsigned long & ns, vpHomogeneousMatrix & dMc, prCartesian3DPointVec & XS, double & rho)
     * \brief implementation of the changeSampleFrame method to update both the sample location and the depth
     * \param ns 
     * \param dMc 
     * \param XS 
     * \param rho 
     * \return 0
     */
		int changeSampleFrame(unsigned long & ns, vpHomogeneousMatrix & dMc, prCartesian3DPointVec & XS, double & rho)
		{
			/*if(isGe3DSpaceSet)
			{*/
				//TODO: test ns?
				prCartesian3DPointVec X = ((prCartesian3DPointVec *)ge3DSpace)[ns].changeFrame(dMc);
				
				rho = sqrt(X.get_X()*X.get_X() + X.get_Y()*X.get_Y() + X.get_Z()*X.get_Z());
	
				XS.setPoint(X.get_X() / rho, X.get_Y() / rho, X.get_Z() / rho);
			/*}
			else
			{
				XS = ((prCartesian3DPointVec *)ge)[ns];
				rho = 1.;
			}*/

			return 0;
		}
		
		 /*!
     * \fn int getSampleRho(unsigned long & ns, double & rho)
     * \brief accessor to the depth of a spherical point 
     * \param ns 
     * \param rho 
     * \return 0
     */
		int getSampleRho(unsigned long & ns, double & rho)
		{
			//TODO: test ns?
			/*if(isGe3DSpaceSet)
			{*/
				rho = bitmapDepth[ns];
			/*}
			else
			{
				rho = 1.;
			}*/

			return 0;
		}
    
//private:
		TDepth *bitmapDepth; /*!< linear array of pixels of the spherical depthmap */

		prGeometricElement *ge3DSpace; /*!< array of 3D points observed */
		bool isGe3DSpaceSet;

    using prRegularlySampledCSImage<TImage>::subdivLevels; /*!< number of subdivision levels of the sphere */

    //prCartesian3DPointVec *XS; /*!< array of Cartesian spherical points coordinates */

		//prAcquisitionModel type below?
    using prRegularlySampledCSImage<TImage>::ge;
    using prRegularlySampledCSImage<TImage>::nbSamples;
    using prRegularlySampledCSImage<TImage>::bitmap;
    using prRegularlySampledCSImage<TImage>::bitmapf;
    using prRegularlySampledCSImage<TImage>::isBitmapfSet;
    using prRegularlySampledCSImage<TImage>::inttyp;

		
};

#endif  //_PRREGULARLYSAMPLEDCSIMAGEDEPTH_H
