//============================================================================================-----
//== NaturalPoint Tracking Tools API Sample: Accessing Camera, Marker, and Trackable Information
//==
//== This command-line application loads a Tracking Tools Project, lists cameras, and 3d marker
//== count.
//============================================================================================-----

#include <windows.h>
#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// For OpenSceneGraph
////////////////////////////////////////////
#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osgDB/ReadFile> 
#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>
#include <osg/StateSet>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/TexGen>
#include <osg/ShapeDrawable>
#include <iostream>
#include <osgText/Text>
////////////////////////////////////////////
#include "NPTrackingTools.h"


////////////////////////////////////////////
//== Callback prototype ==--

float positionX=10.0f,positionY=4.0f,positionZ=-10.0f;
float positionX_final=0.0f,positionY_final=0.0f,positionZ_final=0.0f;
float positionX_esferaEstatica=0.0f,positionY_esferaEstatica=0.0f,positionZ_esferaEstatica=0.0f;
float positionCaidaX=0.0f,positionCaidaY=0.0f,positionCaidaZ=0.0f;
bool choque=false,toca=false;
float dirx=0.0,diry=0.0,dirz=0.0,r=0.0;
/////////////////////////////////////////////

// Local function prototypes
void CheckResult( NPRESULT result );

// Local constants
const float kRadToDeg = 0.0174532925f;

// Local class definitions
class Point4
{
public:
    Point4( float x, float y, float z, float w );

    float           operator[]( int idx ) const { return mData[idx]; }
    const float*    Data() const { return mData; }

private:
    float           mData[4];
};

class TransformMatrix
{
public:
    TransformMatrix();

    TransformMatrix( float m11, float m12, float m13, float m14,
        float m21, float m22, float m23, float m24,
        float m31, float m32, float m33, float m34,
        float m41, float m42, float m43, float m44 );

    void            SetTranslation( float x, float y, float z );
    void            Invert();

    TransformMatrix operator*( const TransformMatrix &rhs );
    Point4          operator*( const Point4 &v );

    static TransformMatrix RotateX( float rads );
    static TransformMatrix RotateY( float rads );
    static TransformMatrix RotateZ( float rads );

private:
    float           mData[4][4];
};

// Clase para eventos del teclado 

    class myKeyboardEventHandler : public osgGA::GUIEventHandler
    {
    public:
       virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
       virtual void accept(osgGA::GUIEventHandlerVisitor& v)   { v.visit(*this); };
    };

    bool myKeyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
     {
       switch(ea.getEventType())
       {
       case(osgGA::GUIEventAdapter::KEYDOWN):
          {
             switch(ea.getKey())
             {
             case 'r':
                choque=false;
				toca=false;
                return false;
                break;
             default:
                return false;
             } 
          }
       default:
          return false;
       }
    }


DWORD WINAPI iniciarOSG(LPVOID lpParam){
	osgViewer::Viewer viewer;
   viewer.setUpViewInWindow(400, 400, 640, 480); // Set del Size de la Ventana de la Escena.


   // Declaro a un Grupo como la Raiz de la Escena.
   osg::Group* root = new osg::Group();

   // Creo la ESFERA ESTATICA 
   osg::Sphere* esferaEstatica = new osg::Sphere( osg::Vec3(0,0,0), 2.0);
   osg::ShapeDrawable* esferaEstaticaDrawable = new osg::ShapeDrawable(esferaEstatica);
   esferaEstaticaDrawable->setColor( osg::Vec4(0.1, 255.0, 0.1, 0.1) );

   osg::PositionAttitudeTransform* esferaEstaticaXForm = new osg::PositionAttitudeTransform();
   esferaEstaticaXForm->setPosition(osg::Vec3(positionX_esferaEstatica,positionY_esferaEstatica,positionZ_esferaEstatica));

   osg::Geode* esferaEstaticaGeode = new osg::Geode();
   //root->addChild(esferaEstaticaXForm);
   esferaEstaticaXForm->addChild(esferaEstaticaGeode);
   esferaEstaticaGeode->addDrawable(esferaEstaticaDrawable);


 /*  osg::ref_ptr<osg::Node> puntoRef = osgDB::readNodeFile( "axes.osgt" );
   root->addChild(puntoRef);*/

   // Dibujo ESFERA en Movimiento
   osg::Sphere* unitSphere = new osg::Sphere( osg::Vec3(0,0,0), 0.3);
   osg::ShapeDrawable* unitSphereDrawable = new osg::ShapeDrawable(unitSphere);
   unitSphereDrawable->setColor( osg::Vec4(0.0, 0.0, 0.0, 1.0) );

   osg::PositionAttitudeTransform* unitSphereXForm = 
      new osg::PositionAttitudeTransform();
   unitSphereXForm->setPosition(osg::Vec3(positionX,positionY,positionZ));
   osg::Geode* unitSphereGeode = new osg::Geode();
   unitSphereGeode->addDrawable(unitSphereDrawable);
   unitSphereXForm->addChild(unitSphereGeode);
   root->addChild(unitSphereXForm);

   // Esfera direccion
   osg::Geode* puntoDireccionGeode = new osg::Geode();
   osg::Sphere* puntoDireccion = new osg::Sphere( osg::Vec3(0,0,0), 0.4);
   osg::ShapeDrawable* puntoDireccionDrawable = new osg::ShapeDrawable(puntoDireccion);
   puntoDireccionDrawable->setColor( osg::Vec4(255.0, 0.0, 0.0, 1.0) );
   puntoDireccionGeode->addDrawable(puntoDireccionDrawable);

   osg::PositionAttitudeTransform* puntoDireccionXForm = new osg::PositionAttitudeTransform();
   puntoDireccionXForm->setPosition(osg::Vec3(positionX_final,positionY_final,positionZ_final)); 
   

   puntoDireccionXForm->addChild(puntoDireccionGeode);


   root->addChild(puntoDireccionXForm);
   
   

	// Dibujo Linea
    osg::Vec3 sp(0.0,0.0,0.0); 
	osg::Vec3 pX(3.0,0.0,0.0);
	osg::Vec3 pY(0.0,3.0,0.0);
	osg::Vec3 pZ(0.0,0.0,3.0);

	osg::ref_ptr<osg::Geometry> ejeX( new osg::Geometry); 
	osg::ref_ptr<osg::Geometry> ejeY( new osg::Geometry);
	osg::ref_ptr<osg::Geometry> ejeZ( new osg::Geometry); 

	osg::ref_ptr<osg::Vec3Array> pointsX = new osg::Vec3Array; 
	pointsX->push_back(sp); 
	pointsX->push_back(pX);
	osg::ref_ptr<osg::Vec3Array> pointsY = new osg::Vec3Array; 
	pointsY->push_back(sp); 
	pointsY->push_back(pY);
	osg::ref_ptr<osg::Vec3Array> pointsZ = new osg::Vec3Array; 
	pointsZ->push_back(sp); 
	pointsZ->push_back(pZ);

	osg::ref_ptr<osg::Vec4Array> colorX = new osg::Vec4Array; 
	colorX->push_back(osg::Vec4(1.0,0.0,0.0,1.0)); 
	osg::ref_ptr<osg::Vec4Array> colorY = new osg::Vec4Array; 
	colorY->push_back(osg::Vec4(0.0,1.0,0.0,1.0));
	osg::ref_ptr<osg::Vec4Array> colorZ = new osg::Vec4Array; 
	colorZ->push_back(osg::Vec4(0.0,0.0,1.0,1.0)); 

	ejeX->setVertexArray(pointsX.get()); 
	ejeX->setColorArray(colorX.get()); 
	ejeY->setVertexArray(pointsY.get()); 
	ejeY->setColorArray(colorY.get()); 
	ejeZ->setVertexArray(pointsZ.get()); 
	ejeZ->setColorArray(colorZ.get()); 

	ejeX->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE); 
	ejeX->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));
	ejeY->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE); 
	ejeY->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));
	ejeZ->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE); 
	ejeZ->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));



	osg::ref_ptr<osg::Geode> geodeAxe (new osg::Geode);

	geodeAxe->addDrawable(ejeX);
	geodeAxe->addDrawable(ejeY);
	geodeAxe->addDrawable(ejeZ);

	osg::ref_ptr<osgText::Text> labX (new osgText::Text()) ;
	osg::ref_ptr<osgText::Text> labY (new osgText::Text()) ;
	osg::ref_ptr<osgText::Text> labZ (new osgText::Text()) ;

	labX->setText("+X");
	labY->setText("+Y");
	labZ->setText("+Z");

	labX->setPosition(pX);
	labY->setPosition(pY);
	labZ->setPosition(pZ);
	labX->setCharacterSize(0.5);
	labY->setCharacterSize(0.5);
	labZ->setCharacterSize(0.5);


	geodeAxe->addDrawable(labX);
	geodeAxe->addDrawable(labY);
	geodeAxe->addDrawable(labZ);

	root->addChild(geodeAxe);



   // Dibujo PLANO PISO
		// Declaro los vertices del Piso
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	//vertices->push_back( osg::Vec3(-20.0f, -20.0f,0.0f ) );
	//vertices->push_back( osg::Vec3(20.0f, -20.0f, 0.0f) );
	//vertices->push_back( osg::Vec3(20.0f,20.0f, 0.0f) );
	//vertices->push_back( osg::Vec3(-20.0f, 20.0f, 0.0f) );
	vertices->push_back( osg::Vec3(0.0f, 0.0f,0.0f ) );
	vertices->push_back( osg::Vec3(6.44f, 0.0f, 0.0f) );
	vertices->push_back( osg::Vec3(6.44f,0.0f, -4.08f) );
	vertices->push_back( osg::Vec3(0.0f, 0.0f, -4.08f) );


		// Declaro la normal
	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
	normals->push_back( osg::Vec3(0.0f,1.0f, 0.0f) );

		// Colores en los vertices
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	//colors->push_back( osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) );
	//colors->push_back( osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f) );
	//colors->push_back( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );

	osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;
	quad->setVertexArray( vertices.get() );
	quad->setNormalArray( normals.get() );
	quad->setNormalBinding( osg::Geometry::BIND_OVERALL );
	quad->setColorArray( colors.get() );
	quad->setColorBinding( osg::Geometry::BIND_OVERALL );

	quad->addPrimitiveSet( new osg::DrawArrays(GL_QUADS, 0, 4) );
	osg::ref_ptr<osg::Geode> rootPlane = new osg::Geode;
	rootPlane->addDrawable( quad.get() );
		// Lo agrego al Root principal de la Escena
	root->addChild(rootPlane);


	// Creo Bounding Spheres de las Figuras en la scene. Esto es para las colisiones.
	// Los Bounding Spheres se esta colocando en las mismas posiciones y mismo radio que las esferas.
	osg::BoundingSphere bs_unitSphere = osg::BoundingSphere(osg::Vec3f(positionX_final,positionY_final,positionZ_final),0.3f);
    osg::BoundingSphere bs2_esferaEstatica = osg::BoundingSphere(osg::Vec3f(positionX_esferaEstatica,positionY_esferaEstatica,positionZ_esferaEstatica),2.0f);


	osg::ref_ptr<osg::LightSource> lightSource (new osg::LightSource);
	lightSource->getLight()->setPosition(osg::Vec4(3.0, 10.0, 3.0,1.0));
	lightSource->getLight()->setAmbient(osg::Vec4(0.2, 0.2, 0.2, 1));
	lightSource->getLight()->setDiffuse(osg::Vec4(0.8, 0.8, 0.8, 1));
	root->addChild(lightSource);


   // OSG1 viewer.setUpViewer(osgProducer::Viewer::STANDARD_SETTINGS);

   viewer.setCameraManipulator(new osgGA::TrackballManipulator());
   //viewer.getCamera()->setProjectionMatrix(osg::Matrix::ortho2D(0,1280,0,1024));
   ///viewer.getCamera()->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
   //viewer.getCamera()->setViewMatrix(osg::Matrix::identity());

   viewer.setSceneData( root );
   viewer.realize();

   myKeyboardEventHandler* myFirstEventHandler = new myKeyboardEventHandler();
   viewer.addEventHandler(myFirstEventHandler); 


   while( !viewer.done() )
   {
	  
	   if((puntoDireccionDrawable->getBound()).intersects(quad->getBound()))
		{
			choque=true;
			//printf("Colision");
			//puntoDireccionDrawable->setColor( osg::Vec4(255.0, 255.1, 0.1, 0.1) );
			//if(!toca)
			//{
			//	positionCaidaX=positionX;
			//	positionCaidaY=positionY;
			//	positionCaidaZ=positionZ;
			//}
			//toca=true;
		}
	  else
		    puntoDireccionDrawable->setColor( osg::Vec4(255.0, 0.0, 0.0, 0.1) );

		//positionZ = positionZ + 0.01f;
		//positionX = positionX - 0.01f;
		unitSphereXForm->setPosition(osg::Vec3(positionX,positionY,positionZ));
		puntoDireccionXForm->setPosition(osg::Vec3(positionX_final,positionY_final,positionZ_final));
		bs_unitSphere.set(osg::Vec3f(positionX,positionY,positionZ),0.3f);
		esferaEstaticaDrawable->setColor( osg::Vec4(0.1, 255.0, 0.1, 0.1) );
        viewer.frame();
   }
	return 0;
}
// Main application
int main( int argc, char* argv[] )
{

	DWORD dwThreadId, dwThrdParam = 1;
	HANDLE hiloParaTracking;

	//Creacion del hilo para Tracking
			hiloParaTracking = CreateThread(
			NULL, // default security attributes
			0, // use default stack size
			iniciarOSG, // thread function
			&dwThrdParam, // argument to thread function
			0, // use default creation flags
			&dwThreadId); // returns the thread identifier
			if (hiloParaTracking == NULL)
				printf("CreateThread() failed, error: %d.\n", GetLastError());
			//else, gives some prompt...
			else
			{
				printf("Creacion del hilo de tracking exitosa\n");
				printf("The thread ID: %u.\n", dwThreadId);
			}
			if (CloseHandle(hiloParaTracking) != 0)
				printf("Handle to thread closed successfully.\n");

			printf("== NaturalPoint Tracking Tools API Marker Sample =======---\n");
    printf("== (C) NaturalPoint, Inc.\n\n");

    printf("Initializing NaturalPoint Devices\n");
    TT_Initialize();

    // Do an update to pick up any recently-arrived cameras.
    TT_Update();

    // Load a project file from the executable directory.
    printf( "Loading Project: project.ttp\n\n" );
    CheckResult( TT_LoadProject("project.ttp") );

    // List all detected cameras.
    printf( "Cameras:\n" );
    for( int i = 0; i < TT_CameraCount(); i++)
    {
        printf( "\t%s\n", TT_CameraName(i) );
    }
    printf("\n");

    // List all defined rigid bodies.
    printf("Rigid Bodies:\n");
    for( int i = 0; i < TT_TrackableCount(); i++)
    {
        printf("\t%s\n", TT_TrackableName(i));
    }
    printf("\n");

    int frameCounter = 0;

    // Poll API data until the user hits a keyboard key.
    while( !_kbhit() )
    {
        if( TT_Update() == NPRESULT_SUCCESS )
        {
            frameCounter++;

            // Update tracking information every 100 frames (for example purposes).
            if( (frameCounter%25) == 0 )
            {
                float   yaw,pitch,roll;
                float   x,y,z;
                float   qx,qy,qz,qw;
                bool    tracked;

                printf( "Frame #%d: (Markers: %d)\n", frameCounter, TT_FrameMarkerCount() );

                for( int i = 0; i < TT_TrackableCount(); i++ )
                {
                    TT_TrackableLocation( i, &x,&y,&z, &qx,&qy,&qz,&qw, &yaw,&pitch,&roll );

                    if( TT_IsTrackableTracked( i ) )
                    {
                        printf( "\t%s: Pos (%.3f, %.3f, %.3f) Orient (%.1f, %.1f, %.1f)\n", TT_TrackableName( i ),
                            x, y, z, yaw, pitch, roll );
						/////////////////////////////////////////////////////////////////////////
						
							positionX=x*10.0f;
							positionY=y*10.0f;
							positionZ=z*-10.0f;

	dirx= 2*(qx * qy - qz*qw);
	diry = 1-2*(qx*qx+qz*qz);
	dirz= 2*(qy*qz+qx*qw);

	r=1;
	float x1= positionX-(dirx/r);
	float y1= positionY-(diry/r);
	float z1=positionZ-(dirz/r);

	float a=-x1+positionX; // vector direccion
	float b=-y1+positionY;
	float c=-z1+positionZ;
	 printf("Tracker Position:(%.4f,%.4f,%.4f) Orientation:(%.2f,%.2f,%.2f,%.2f) yInter:%.2f \n",
        x, y, z,
        qx, qy, qz, qw,b);
	if(b!=0.0){
		float t=-positionY/b;
		positionX_final=positionX+t*a;
		positionY_final=0.0;
		positionZ_final=positionZ+t*c;

		 /// Sacar posicion punta de la pluma
		float largo_pluma = 0.135*10.0f;
		float k = sqrt((largo_pluma*largo_pluma) / (a*a + b*b + c*c));
		float h = positionY + k*b;
		positionX_final=positionX+k*a;
		positionY_final=positionY + k*b;
		positionZ_final=positionZ+k*c;
		 printf("Punta Position:(%.4f,%.4f,%.4f) \n",
       positionX_final,positionY_final,positionZ_final);

						
	}
						//positionX_final=x*10.0f;
						//positionY_final=y*10.0f;
						//positionZ_final=z*-10.0f;
						/////////////////////////////////////////////////////////////////////////



                        TransformMatrix xRot( TransformMatrix::RotateX( -roll * kRadToDeg ) );
                        TransformMatrix yRot( TransformMatrix::RotateY( -yaw * kRadToDeg ) );
                        TransformMatrix zRot( TransformMatrix::RotateZ( -pitch * kRadToDeg ) );

                        // Compose the local-to-world rotation matrix in XZY (roll, pitch, yaw) order.
                        TransformMatrix worldTransform = xRot * zRot * yRot;

                        // Inject world-space coordinates of the origin.
                        worldTransform.SetTranslation( x, y, z );

                        // Invert the transform matrix to convert from a local-to-world to a world-to-local.
                        worldTransform.Invert();

                        float   mx, my, mz;
                        int     markerCount = TT_TrackableMarkerCount( i );
                        for( int j = 0; j < markerCount; ++j )
                        {
                            // Get the world-space coordinates of each rigid body marker.
                            TT_TrackablePointCloudMarker( i, j, tracked, mx, my, mz );

                            // Transform the rigid body point from world coordinates to local rigid body coordinates.
                            // Any world-space point can be substituted here to transform it into the local space of
                            // the rigid body.
                            Point4  worldPnt( mx, my, mz, 1.0f );
                            Point4  localPnt = worldTransform * worldPnt;

                            printf( "\t\t%d: World (%.3f, %.3f, %.3f) Local (%.3f, %.3f, %.3f)\n", j + 1, 
                                mx, my, mz, localPnt[0], localPnt[1], localPnt[2] );
                        }
                    }
                    else
                    {
                        printf( "\t%s: Not Tracked\n", TT_TrackableName( i ) );
                    }
                }
            }
        }
        Sleep(2);
    }

    printf( "Shutting down NaturalPoint Tracking Tools\n" );
    CheckResult( TT_Shutdown() );

    printf( "Complete\n" );
    while( !_kbhit() )
    {
        Sleep(20);
    }

    TT_FinalCleanup();
	 getchar();
	return 0;
		
    
}


void CheckResult( NPRESULT result )   //== CheckResult function will display errors and ---
                                      //== exit application after a key is pressed =====---
{
    if( result!= NPRESULT_SUCCESS)
    {
        // Treat all errors as failure conditions.
        printf( "Error: %s\n\n(Press any key to continue)\n", TT_GetResultString(result) );

        Sleep(20);
		 getchar();
        exit(1);
    }
}

//
// Point4
//

Point4::Point4( float x, float y, float z, float w )
{
    mData[0] = x;
    mData[1] = y;
    mData[2] = z;
    mData[3] = w;
}

//
// TransformMatrix
//

TransformMatrix::TransformMatrix()
{
    for( int i = 0; i < 4; ++i )
    {
        for( int j = 0; j < 4; ++j )
        {
            if( i == j )
            {
                mData[i][j] = 1.0f;
            }
            else
            {
                mData[i][j] = 0.0f;
            }
        }
    }
}

TransformMatrix::TransformMatrix( float m11, float m12, float m13, float m14,
    float m21, float m22, float m23, float m24,
    float m31, float m32, float m33, float m34,
    float m41, float m42, float m43, float m44 )
{
    mData[0][0] = m11;
    mData[0][1] = m12;
    mData[0][2] = m13;
    mData[0][3] = m14;
    mData[1][0] = m21;
    mData[1][1] = m22;
    mData[1][2] = m23;
    mData[1][3] = m24;
    mData[2][0] = m31;
    mData[2][1] = m32;
    mData[2][2] = m33;
    mData[2][3] = m34;
    mData[3][0] = m41;
    mData[3][1] = m42;
    mData[3][2] = m43;
    mData[3][3] = m44;
}

void TransformMatrix::SetTranslation( float x, float y, float z )
{
    mData[0][3] = x;
    mData[1][3] = y;
    mData[2][3] = z;
}

void TransformMatrix::Invert()
{
    // Exploit the fact that we are dealing with a rotation matrix + translation component.
    // http://stackoverflow.com/questions/2624422/efficient-4x4-matrix-inverse-affine-transform

    float   tmp;
    float   vals[3];

    // Transpose left-upper 3x3 (rotation) sub-matrix
    tmp = mData[0][1]; mData[0][1] = mData[1][0]; mData[1][0] = tmp;
    tmp = mData[0][2]; mData[0][2] = mData[2][0]; mData[2][0] = tmp;
    tmp = mData[1][2]; mData[1][2] = mData[2][1]; mData[2][1] = tmp;

    // Multiply translation component (last column) by negative inverse of upper-left 3x3.
    for( int i = 0; i < 3; ++i )
    {
        vals[i] = 0.0f;
        for( int j = 0; j < 3; ++j )
        {
            vals[i] += -mData[i][j] * mData[j][3];
        }
    }
    for( int i = 0; i < 3; ++i )
    {
        mData[i][3] = vals[i];
    }
}

TransformMatrix TransformMatrix::RotateX( float rads )
{
    return TransformMatrix( 1.0, 0.0, 0.0, 0.0,
        0.0, cos( rads ), -sin( rads ), 0.0,
        0.0, sin( rads ), cos( rads ), 0.0,
        0.0, 0.0, 0.0, 1.0 );
}

TransformMatrix TransformMatrix::RotateY( float rads )
{
    return TransformMatrix( cos( rads ), 0.0, sin( rads ), 0.0,
        0.0, 1.0, 0.0, 0.0,
        -sin( rads ), 0.0, cos( rads ), 0.0,
        0.0, 0.0, 0.0, 1.0 );
}

TransformMatrix TransformMatrix::RotateZ( float rads )
{
    return TransformMatrix( cos( rads ), -sin( rads ), 0.0, 0.0,
        sin( rads ), cos( rads ), 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0 );
}

TransformMatrix TransformMatrix::operator*( const TransformMatrix &rhs )
{
    TransformMatrix result;

    for( int i = 0; i < 4; ++i )
    {
        for( int j = 0; j < 4; ++j )
        {
            float rowCol = 0.0;
            for( int k = 0; k < 4; ++k )
            {
                rowCol += mData[i][k] * rhs.mData[k][j];
            }
            result.mData[i][j] = rowCol;
        }
    }
    return result;
}

Point4 TransformMatrix::operator*( const Point4 &v )
{
    const float *pnt = v.Data();
    float   result[4];

    for( int i = 0; i < 4; ++i )
    {
        float rowCol = 0.0;
        for( int k = 0; k < 4; ++k )
        {
            rowCol += mData[i][k] * pnt[k];
        }
        result[i] = rowCol;
    }
    return Point4( result[0], result[1], result[2], result[3] );
}
