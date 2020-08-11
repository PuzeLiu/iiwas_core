/**

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," 
without warranty of any kind, including without limitation the warranties 
of merchantability, fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable 
for any particular purpose. In no event shall KUKA be responsible for loss 
or damages arising from the installation or use of the Software, 
including but not limited to any indirect, punitive, special, incidental 
or consequential damages of any character including, without limitation, 
damages for loss of goodwill, work stoppage, computer failure or malfunction, 
or any and all other commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by KUKA. 
Should the Software prove defective, KUKA is not liable for the entire cost 
of any service and repair.


COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2016 
KUKA Roboter GmbH
Augsburg, Germany

This material is the exclusive property of KUKA Roboter GmbH and must be returned 
to KUKA Roboter GmbH immediately upon request.  
This material and the information illustrated or contained herein may not be used, 
reproduced, stored in a retrieval system, or transmitted in whole 
or in part in any way - electronic, mechanical, photocopying, recording, 
or otherwise, without the prior written consent of KUKA Roboter GmbH.  






\file
\version {1.10}
*/
#ifndef _KUKA_FRI_TRANSFORMATION_CLIENT_H
#define _KUKA_FRI_TRANSFORMATION_CLIENT_H

#include <vector>

/** Kuka namespace */
namespace KUKA
{
namespace FRI
{

	// forward declaration
	struct ClientData;
	class TransformationContainer;

   /**
    * \brief Abstract FRI transformation client. 
    * 
    * A transformation client allows the user to send transformation matrices cyclically to the 
    * KUKA Sunrise controller for manipulating the transformation of a dynamic frame. 
    * These matrices can be provided by an external sensor. 
    * <br>
    * Custom transformation clients have to derived from this class. 
    * The callback provide() is cyclically called by the client application
    * (ClientApplication) whenever new FRI messages arrive.
    * 
    * <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b> 
    */
   class TransformationClient
   {
      
      friend class ClientApplication;
     
   public:
      
      /**
       * <br>  <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>  <br> 
       *  \brief Constructor
       **/
      TransformationClient();
      
      /** <br> <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>   <br> 
       * \brief Virtual destructor. 
       **/
      virtual ~TransformationClient();
      
      /** 
       * <br>  <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>  <br>
       * \brief Callback which is called whenever a new FRI message arrives. 
       *
       *  In this callback all requested transformations have to be set.
       *  
       *  \see getRequestedTransformationIDs(), setTransformation()
       *  
       */
      virtual void provide() = 0;
      
      
   protected:

      /** 
       * <br>  <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>  <br>
       * \brief  Returns a vector of identifiers of requested transformation matrices.
       * 
       * The custom TransformationClient has to provide data for transformation matrices with these
       * identifiers. 
       * 
       * 
       * @return vector of IDs of requested transformations
       */
      const std::vector<const char*>* getRequestedTransformationIDs() const;
      
      /** 
       * <br>  <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>  <br>
       * 
       * \brief Get the timestamp of the current received FRI monitor message in Unix time. 
       * 
       * This method returns the seconds since 0:00, January 1st, 1970 (UTC).
       * Use getTimestampNanoSec() to increase your timestamp resolution when
       * seconds are insufficient. 
       * 
       * 
       * 
       * @return timestamp encoded as Unix time (seconds)
       */
      const unsigned int getTimestampSec() const;
      
      /** 
       * <br>  <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>  <br>
       * \brief Get the nanoseconds elapsed since the last second (in Unix time).
       * 
       * This method complements getTimestampSec() to get a high resolution 
       * timestamp. 
       * 
       * @return timestamp encoded as Unix time (nanoseconds part)
       */
      const unsigned int getTimestampNanoSec() const;
      
      /**
       * <br>  <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>  <br>
       * \brief Set the transformation matrix of the current cyclic step.
       * 
       * A transformation matrix has 3x4 elements. It consists of a rotational matrix (3x3 elements)
       * and a translational vector (3x1 elements). The complete transformation matrix has the
       * following structure: <br>
       * [Transformation(3x4)] = [Rotation(3x3) | Translation(3x1) ]
       * <p>
       * All provided transformation matrices need a timestamp for which they are valid. 
       * The timestamp handed over to this function must base on the timestamp provided by
       * the robot controller (see getTimestampSec(), getTimestampNanoSec()).
       * <p>
       * If the callback provide() is called more often than a new transformation matrix has been
       * calculated, just set again the last calculated transformation matrix together with the 
       * last used timestamp.
       *  
       * @param transformationID Identifier string of the transformation matrix
       * @param transformationMatrix Provided transformation matrix
       * @param timeSec Timestamp encoded as Unix time (seconds)
       * @param timeNanoSec Timestamp encoded as Unix time (nanoseconds part)
       * 

       * 
       */
      void setTransformation(const char* transformationID, 
            double transformationMatrix[3][4], unsigned int timeSec, unsigned int timeNanoSec);
      
   private:     
      TransformationContainer* _data; //!< container for the data of the transformation
      
      /**
       * \brief Method to create and initialize the client data structure (used internally).
       * 
       * @return newly allocated client data structure
       */
      ClientData* createData();
      
      /**
       * \brief
       * @param clientData
       */
      void linkData(ClientData* clientData);
      
      /**
       * \brief Forbidden copy constructor.
       */
      TransformationClient(const TransformationClient& obj);
   };
   
}
}

#endif // _KUKA_FRI_TRANSFORMATION_CLIENT_H
