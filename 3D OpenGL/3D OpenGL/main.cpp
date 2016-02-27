//
//  main.cpp
//  3D OpenGL
//
//  Created by Joshua Pitkofsky on 11/26/15.
//  Copyright © 2015 Joshua Pitkofsky. All rights reserved.
//

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <iostream>
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
// Needed on MsWindows
#include <windows.h>
#endif // Win32 platform

#include <openGL/gl.h>
#include <openGL/glu.h>
// Download glut from: http://www.opengl.org/resources/libraries/glut/
#include <GLUT/glut.h>
#include "Mesh.h"
#include "float2.h"
#include "float3.h"
#include <vector>
#include <map>
extern "C" unsigned char* stbi_load(char const *filename, int *x, int *y, int *comp, int req_comp);
bool twoPlayer = false;
class LightSource
{
public:
    virtual float3 getRadianceAt  ( float3 x )=0;
    virtual float3 getLightDirAt  ( float3 x )=0;
    virtual float  getDistanceFrom( float3 x )=0;
    virtual void   apply( GLenum openglLightName )=0;
};

class DirectionalLight : public LightSource
{
    float3 dir;
    float3 radiance;
public:
    DirectionalLight(float3 dir, float3 radiance)
    :dir(dir), radiance(radiance){}
    float3 getRadianceAt  ( float3 x ){return radiance;}
    float3 getLightDirAt  ( float3 x ){return dir;}
    float  getDistanceFrom( float3 x ){return 900000000;}
    void   apply( GLenum openglLightName )
    {
        float aglPos[] = {dir.x, dir.y, dir.z, 0.0f};
        glLightfv(openglLightName, GL_POSITION, aglPos);
        float aglZero[] = {0.0f, 0.0f, 0.0f, 0.0f};
        glLightfv(openglLightName, GL_AMBIENT, aglZero);
        float aglIntensity[] = {radiance.x, radiance.y, radiance.z, 1.0f};
        glLightfv(openglLightName, GL_DIFFUSE, aglIntensity);
        glLightfv(openglLightName, GL_SPECULAR, aglIntensity);
        glLightf(openglLightName, GL_CONSTANT_ATTENUATION, 1.0f);
        glLightf(openglLightName, GL_LINEAR_ATTENUATION, 0.0f);
        glLightf(openglLightName, GL_QUADRATIC_ATTENUATION, 0.0f);
    }
};

class PointLight : public LightSource
{
    float3 pos;
    float3 power;
public:
    PointLight(float3 pos, float3 power)
    :pos(pos), power(power){}
    float3 getRadianceAt  ( float3 x ){return power*(1/(x-pos).norm2()*4*3.14);}
    float3 getLightDirAt  ( float3 x ){return (pos-x).normalize();}
    float  getDistanceFrom( float3 x ){return (pos-x).norm();}
    void   apply( GLenum openglLightName )
    {
        float aglPos[] = {pos.x, pos.y, pos.z, 1.0f};
        glLightfv(openglLightName, GL_POSITION, aglPos);
        float aglZero[] = {0.0f, 0.0f, 0.0f, 0.0f};
        glLightfv(openglLightName, GL_AMBIENT, aglZero);
        float aglIntensity[] = {power.x, power.y, power.z, 1.0f};
        glLightfv(openglLightName, GL_DIFFUSE, aglIntensity);
        glLightfv(openglLightName, GL_SPECULAR, aglIntensity);
        glLightf(openglLightName, GL_CONSTANT_ATTENUATION, 0.0f);
        glLightf(openglLightName, GL_LINEAR_ATTENUATION, 0.0f);
        glLightf(openglLightName, GL_QUADRATIC_ATTENUATION, 0.25f / 3.14f);
    }
};

class Material
{
public:
    float3 kd;			// diffuse reflection coefficient
    float3 ks;			// specular reflection coefficient
    float shininess;	// specular exponent
    Material()
    {
        kd = float3(0.5, 0.5, 0.5) + float3::random() * 0.5;
        ks = float3(1, 1, 1);
        shininess = 15;
    }
    virtual void apply()
    {
        float aglDiffuse[] = {kd.x, kd.y, kd.z, 1.0f};
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, aglDiffuse);
        float aglSpecular[] = {kd.x, kd.y, kd.z, 1.0f};
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, aglSpecular);
        if(shininess <= 128)
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
        else
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128.0f);
        glDisable(GL_TEXTURE_2D);
    }
};

class Camera
{
    float3 eye;
    
    float3 ahead;
    float3 lookAt;
    float3 right;
    float3 up;
    
    float fov;
    float aspect;
    
    float2 lastMousePos;
    float2 mouseDelta;
    
public:
    float3 getEye()
    {
        return eye;
    }
    Camera()
    {
        eye = float3(0, 0, -5);
        lookAt = float3(0, 0, 0);
        right = float3(1, 0, 0);
        up = float3(0, 1, 0);
        fov = 1.1;
        aspect  = 1;
    }
    
    void apply()
    {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(fov /3.14*180, aspect, 0.1, 500);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(eye.x, eye.y, eye.z, lookAt.x, lookAt.y, lookAt.z, 0.0, 1.0, 0.0);
    }
    
    void setAspectRatio(float ar) { aspect= ar; }
    
    void move(float dt, std::vector<bool>& keysPressed, float3 avatarPos, float avatarOrient)
    {

 
        float3 ahead = avatarPos - float3(-cos((avatarOrient*M_PI)/180)*5, 0, sin((avatarOrient*M_PI)/180)*5 );
        eye = float3(ahead.x, ahead.y+2, ahead.z);
        
//        right = ahead.cross(float3(0, 3, 0)).normalize();
//        up = right.cross(ahead);
     
        lookAt = avatarPos;
        if (twoPlayer) {
            eye = float3(100,100,100);
            lookAt =(float3(0,0,0));
        }
    }
    
    void startDrag(int x, int y)
    {
        lastMousePos = float2(x, y);
    }
    void drag(int x, int y)
    {
        float2 mousePos(x, y);
        mouseDelta = mousePos - lastMousePos;
        lastMousePos = mousePos;
    }
    void endDrag()
    {
        mouseDelta = float2(0, 0);
    }
    
};

extern "C" unsigned char* stbi_load(
                                    char const *filename,
                                    int *x, int *y,
                                    int *comp, int req_comp);
class Metal : public Material {
    float3 r0;
public:
    Metal(float3  refractiveIndex, float3  extinctionCoefficient){
        float3 rim1 = refractiveIndex - float3(1,1,1);
        float3 rip1 = refractiveIndex + float3(1,1,1);
        float3 k2 = extinctionCoefficient * extinctionCoefficient;
        r0 = (rim1*rim1 + k2)/ (rip1*rip1 + k2);
    }
    
    float3 shade(
                 float3 normal,
                 float3 viewDir,
                 float3 lightDir,
                 float3 lightPowerDensity, float3 face)
    {
        return  float3(0,0,0);
    }
    struct Event{
        float3 reflectionDir;
        float3 reflectance;
        
    };
    Event evaluateEvent(float3 inDir, float3 normal) {
        Event e;
        float cosa = -normal.dot(inDir);
        float3 perp = -normal * cosa;
        float3 parallel = inDir - perp;
        e.reflectionDir = parallel - perp;
        e.reflectance = r0 + (float3(1,1,1)-r0) * pow(1 - cosa, 5);
        return e; }
};



class Object
{
public:
    bool flag = false;
protected:
    Material* material;
    float3 scaleFactor;
    float3 position;
    float3 orientationAxis;
    float orientationAngle;
  

public:
    Object(Material* material):material(material),orientationAngle(0.0f),scaleFactor(1.0,1.0,1.0),orientationAxis(0.0,1.0,0.0){
    }
    
    virtual ~Object(){}
    
    Object* translate(float3 offset){
        position += offset; return this;
    }
    Object* scale(float3 factor){
        scaleFactor *= factor; return this;
    }
    Object* rotate(float angle){
        orientationAngle += angle; return this;
    }
    float3 getPosition(){
        return position;
    }
    void setYPosition(float positiona){
        position.y = positiona;
    }
    virtual bool flagDelete(){
        flag=true;
        return false;
    }
    
    virtual void draw()
    {
        material->apply();
        // apply scaling, translation and orientation
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glTranslatef(position.x, position.y, position.z);
        glRotatef(orientationAngle, orientationAxis.x, orientationAxis.y, orientationAxis.z);
        glScalef(scaleFactor.x, scaleFactor.y, scaleFactor.z);
        drawModel();
        glPopMatrix();
    }
    virtual float getSize(){
        return 0;
    }


        virtual void drawShadow(float3 lightDir)
    {

        // apply scaling, translation and orientation
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        
        glTranslatef(0, 0.5, 0);
        glScalef(1, 0, 1);
        
        glTranslatef(position.x, position.y, position.z);
        glRotatef(orientationAngle, orientationAxis.x, orientationAxis.y, orientationAxis.z);
        
        glScalef(scaleFactor.x, scaleFactor.y, scaleFactor.z);
        drawModel();
        glPopMatrix();
    }
    
    virtual void drawModel()=0;
    virtual void move(double t, double dt){}
    virtual bool control(std::vector<bool>& keysPressed, std::vector<Object*>& spawn, std::vector<Object*>& objects, std::vector<Object*>& bullets, std::vector<LightSource*>& lightSources, std::vector<Object*>& platforms, std::vector<Object*>& enemies, std::vector<Object*>& enemyBullets){return false;}
};


class Teapot : public Object
{
public:
    Teapot(Material* material):Object(material){}
    void drawModel()
    {
        glutSolidTeapot(1.0f);
    }
};

class MeshInstance : public Object
{
    Mesh* meshy;
public:
    MeshInstance(Mesh* mesh, Material* material):Object(material),meshy(mesh){}
    void drawModel()
    {
        meshy->draw();
    }
};

class Ground : public Object
{
   
public:
    Ground( Material* material):Object(material){}
    void drawModel()
    {
        glBegin(GL_QUADS);
       // glTexCoord2d( 0, 1.0);
        glVertex3d( 1000,  0, 1000);
        //glTexCoord2d( 1.0, 1.0);
        glVertex3d( 1000, 0,-1000);
        //glTexCoord2d( 0.0, 2.0);
        glVertex3d(-1000,  0,-1000);
        //glTexCoord2d( 2.0, 2.0);
        glVertex3d(-1000, 0,1000);
        glEnd();
    }
    void drawShadow(float3 lightDir)
    {
        return;
    }

};

class Sky : public Object
{
    
public:
    Sky( Material* material):Object(material){}
    void drawModel()
    {
        glBegin(GL_QUAD_STRIP);
        glTexCoord2d( 0, 1.0);
        glVertex3d( -200,  -1000, 200);
        glTexCoord2d( 1.0, 1.0);
        glVertex3d( -200, 1000,200);
        
        glTexCoord2d( 0.0, 2.0);
        glVertex3d(200,  -1000,200);
        glTexCoord2d( 2.0, 2.0);
        glVertex3d(200, 1000,200);
        
        glTexCoord2d( 0, 3.0);
        glVertex3d( 200, -1000, -200);
        glTexCoord2d( 1.0, 1.0);
        glVertex3d( 200, 1000,-200);
        
        glTexCoord2d( 3.0, 3.0);
        glVertex3d(-200,  -1000,-200);
        glTexCoord2d( 0.0, 0.0);
        glVertex3d(-200,1000,-200);

        glTexCoord2d( 0, 4.0);
        glVertex3d( -200,  -1000, 200);
        glTexCoord2d( 4.0, 4.0);
        glVertex3d( -200, 1000,200);
        glEnd();
        
    
    }
    void drawShadow(float3 lightDir)
    {
        return;
    }
    
};


class Bullet : public Object
{
    float3 velocity;
    float3 orientation;
    float age=0;
    
public:
    
    Bullet(float3 startPoint, float3 orientation, Material* material):Object(material){
        velocity = orientation.normalize()*10;
        position = startPoint;
        
        
    }
    void drawModel()
    {
        
        glutWireSphere(3, 11, 5);
        //glutWireTeapot(3);
    }
    
    void move(double t, double dt){
        position+=velocity*dt;
        age+=dt;
    }
    
    
    bool control(std::vector<bool>& keysPressed, std::vector<Object*>& spawn, std::vector<Object*>& objects, std::vector<Object*>& bullets, std::vector<LightSource*>& lightSources, std::vector<Object*>& platforms, std::vector<Object*>& enemies, std::vector<Object*>& enemyBullets){

  

        if (age>4){
            return true;
        }
        
        return false;
    }
};
float3 reflect(  	float3 inDir,
               float3 normal)
{
    float cosa = -normal.dot(inDir);
    float3 perp = -normal * cosa;
    float3 parallel = inDir - perp;
    return parallel - perp;
};
class Bullets : public Object
{
    float3 velocity;
    float3 orientation;
    float age=0;
    
public:
    
    Bullets(float3 startPoint, float3 orientation, Material* material):Object(material){
        velocity = orientation.normalize()*100;
        position = startPoint;
        
        
    }
    void drawModel()
    {
        glutSolidIcosahedron();
    

    }
    
    void move(double t, double dt){
        position+=velocity*dt;
          age+=dt;
    }
    
    
    bool control(std::vector<bool>& keysPressed, std::vector<Object*>& spawn, std::vector<Object*>& objects, std::vector<Object*>& bullets, std::vector<LightSource*>& lightSources, std::vector<Object*>& platforms, std::vector<Object*>& enemies, std::vector<Object*>& enemyBullets){
        
      

        if (age>3){
            return true;
        }
        return false;
    }

};

class EnemyBullet : public Object
{
    float3 velocity;
    float3 orientation;
    float age=0;
    
public:
    
    EnemyBullet(float3 startPoint, float3 orientation, Material* material):Object(material){
        velocity = orientation.normalize()*100;
        position = startPoint;
        
    }
    void drawModel()
    {
        glutWireIcosahedron();
    }
    
    void move(double t, double dt){
        position+=velocity*dt;
          age+=dt;
    }
    
    
    bool control(std::vector<bool>& keysPressed, std::vector<Object*>& spawn, std::vector<Object*>& objects, std::vector<Object*>& bullets, std::vector<LightSource*>& lightSources, std::vector<Object*>& platforms, std::vector<Object*>& enemies, std::vector<Object*>& enemyBullets){
        
      
        
        if (age>3){
            return true;
        }
        
        return false;
    }
};


class Target : public Object
{
    float3 velocity;
    float3 orientation;
    float age=0;
    float size=10;
    
public:
    
    Target(Material* material):Object(material){
        
        
    }
    void drawModel()
    {
        glutSolidTorus(size/2, size*2, size/2,1000/ size);
    }
    
    void move(double t, double dt){
        position+=velocity*dt;
        rotate(orientationAngle);
        
    }
    
    
    bool control(std::vector<bool>& keysPressed, std::vector<Object*>& spawn, std::vector<Object*>& objects, std::vector<Object*>& bullets, std::vector<LightSource*>& lightSources, std::vector<Object*>& platforms, std::vector<Object*>& enemies, std::vector<Object*>& enemyBullets){
        
        
        float threshold = size*2;
        for(int i = 0; i<bullets.size(); i++){
            
            float3 posThat = bullets.at(i)->getPosition();
            
            if (((position - posThat).norm())<threshold*1.1){
                
                size = size*.99;
                
            }
           else if (((position - posThat).norm())<threshold){
                
                size = size*.98;
                
            }
           else if (((position - posThat).norm())<threshold*.8){
               
               size = size*.95;
               
           }
        }
       

                return false;
    }
};

class Platform : public Object
{
    float3 velocity;
    float3 orientation;
    float age=0;
    float size=3;
    
public:
    
    Platform(Material* material):Object(material){
        
        
    }
    void drawModel()
    {
        glutSolidTeapot(size);
      
    }
    
    void move(double t, double dt){
        position+=velocity*dt;
        
    }
    float getSize(){
        return size;
    }
    
    
    bool control(std::vector<bool>& keysPressed, std::vector<Object*>& spawn, std::vector<Object*>& objects, std::vector<Object*>& bullets, std::vector<LightSource*>& lightSources, std::vector<Object*>& platforms, std::vector<Object*>& enemies, std::vector<Object*>& enemyBullets){
        
        
        float threshold = size;
        for(int i = 0; i<bullets.size(); i++){
            
            float3 posThat = bullets.at(i)->getPosition();
            
            if (((position - posThat).norm())<threshold*1.1){
                if(size<10){
                size = size*1.1;
                }
                
            }
            
        }
        return false;
    }
};


class Enemy : public Object
{
    
    float maxVelocity= 500000;
    
    float3 velocity = float3(0, 0, 0);
    float angularVelocity = 0;
    float restitution = 1;
    float angularAcceleration = 0;
    float3 acceleration=float3(0,0,0);
    float cooldownRemaining=0;
    float size=10;
    
    
public:
    Enemy(Material* material):Object(material){}
    
    virtual void move(double t, double dt){
        if (velocity.norm2()>maxVelocity) {
            velocity=velocity/velocity.norm()*maxVelocity;
        }
        else{
            velocity+= acceleration*dt;
            position+=velocity*dt;
            angularVelocity *= pow(0.1, dt);
            velocity *= pow(.99, dt);
            angularVelocity+=angularAcceleration*dt;
            orientationAngle += angularVelocity*dt;
            cooldownRemaining=cooldownRemaining-dt;
            if(position.y < 0) velocity.y *= -restitution;
        }
        
    }
    
    void drawModel()
    {
        glutSolidCube(size);
        
    }
    

    float getSize(){
        return size;
    }
    bool control(std::vector<bool>& keysPressed, std::vector<Object*>& spawn, std::vector<Object*>& objects, std::vector<Object*>& bullets, std::vector<LightSource*>& lightSources, std::vector<Object*>& platforms, std::vector<Object*>& enemies, std::vector<Object*>& enemyBullets){

        float x = (float)rand()/(float)(RAND_MAX/200);
        angularAcceleration=x;
        acceleration = float3(-cos((orientationAngle*M_PI)/180)*10, 0, sin((orientationAngle*M_PI)/180) *10);

 
            
        if(cooldownRemaining<=0) {
            EnemyBullet* bullet = new EnemyBullet(getPosition(), float3(cos(((orientationAngle-180)*M_PI)/180) * 100,-10,-sin(((orientationAngle-180)*M_PI)/180) *100), new Material());
            enemyBullets.push_back(bullet);
            cooldownRemaining=.5;
        }
        
        if(position.x>200){
            velocity.x=-velocity.x;
        }
        else if(position.x<-200){
            velocity.x=-velocity.x;
        }
        else if(position.z>200){
            velocity.z=-velocity.z;
        }
        else if(position.z<-200){
            velocity.z=-velocity.z;
        }
        
        
        float thresholde = 15;
        
        
        for(int i = 0; i<spawn.size(); i++){
            
            float3 posThat = spawn.at(i)->getPosition();
            
            if (((position - posThat).norm())<thresholde){
                acceleration.y = 200;
                size = size*.9;
            }
        }
        

        
        
        float threshold = 10;

        for(int i = 0; i<bullets.size(); i++){
            
            float3 posBullet = bullets.at(i)->getPosition();
                        if (((position - posBullet).norm())<threshold){
                return true;
            }
        }
        
        
        for(int i = 0; i<platforms.size(); i++){
            
            Object* platform = platforms.at(i);
            float platformSize = platform->getSize();
            float3 posThat = float3(platform->getPosition().x,platform->getPosition().y, platform->getPosition().z);
            
            if (((position - posThat).norm())<platformSize){
                velocity.y = abs(velocity.y);
                
            }
        }
        return false;
    }
    
    float getOrientation(){
        return orientationAngle;
    }
    
};


class Bouncer : public MeshInstance
{
  
    float maxVelocity= 500000;
    
    float3 velocity = float3(0, 0, 0);
    float angularVelocity = 0;
    float restitution = 1;
    float angularAcceleration = 0;
    float3 acceleration=float3(0,0,0);
    float cooldownRemaining=0;
    
    
public:
    Bouncer(Mesh* mesh, Material* material):MeshInstance(mesh, material){}
    
    virtual void move(double t, double dt){
        if (velocity.norm2()>maxVelocity) {
            velocity=velocity/velocity.norm()*maxVelocity;
        }
        else{
        velocity+= acceleration*dt;
        position+=velocity*dt;
        angularVelocity *= pow(0.1, dt);
        velocity *= pow(.7, dt);
        angularVelocity+=angularAcceleration*dt;
        orientationAngle += angularVelocity*dt;
        cooldownRemaining=cooldownRemaining-dt;
        if(position.y < 0) velocity.y *= -restitution;
        }
        
    }
    bool control(std::vector<bool>& keysPressed, std::vector<Object*>& spawn, std::vector<Object*>& objects, std::vector<Object*>& bullets, std::vector<LightSource*>& lightSources, std::vector<Object*>& platforms, std::vector<Object*>& enemies, std::vector<Object*>& enemyBullets){
        
        if(keysPressed.at('a'))
            angularAcceleration=200;
        else if(keysPressed.at('d'))
            angularAcceleration=-200;
        
        else
            angularAcceleration=0;
        
        if(keysPressed.at('w'))
            acceleration = float3(-cos((orientationAngle*M_PI)/180)*60, -10, sin((orientationAngle*M_PI)/180) *60);
        else if(keysPressed.at('s'))
            acceleration = float3(cos((orientationAngle*M_PI)/180) *60, -10, -sin((orientationAngle*M_PI)/180) *60);
        else
            acceleration=float3(0,-10,0);
        if(keysPressed['b'])
        {
            
            if(cooldownRemaining<=0){
                Bullet* bullet = new Bullet(getPosition(), float3(0, -5 ,0), new Material());
                spawn.push_back(bullet);
                cooldownRemaining=.5;
            }
        }
            if(keysPressed[' '])
            {
                
                if(cooldownRemaining<=0){
                    Bullets* bullet = new Bullets(getPosition(), float3(cos(((orientationAngle-180)*M_PI)/180) * 60,-5,-sin(((orientationAngle-180)*M_PI)/180) *60), new Material());
                    bullets.push_back(bullet);
                    
                    cooldownRemaining=.5;

                }
        }
        
        
        if(position.x>200){
            velocity.x=-velocity.x;
        }
        else if(position.x<-200){
            velocity.x=-velocity.x;
        }
        else if(position.z>200){
            velocity.z=-velocity.z;
        }
        else if(position.z<-200){
            velocity.z=-velocity.z;
        }

        
        float threshold = 15;
  
        
        for(int i = 0; i<spawn.size(); i++){
            
            float3 posThat = spawn.at(i)->getPosition();
            
            if (((position - posThat).norm())<threshold){
                acceleration.y = 20;
            }
        }
        
        for(int i = 0; i<platforms.size(); i++){
            
            Object* platform = platforms.at(i);
            float platformSize = platform->getSize();
            float3 posThat = float3(platform->getPosition().x,platform->getPosition().y, platform->getPosition().z);
            
            if (((position - posThat).norm())<platformSize){
                velocity.y = abs(velocity.y);
                
            }
        }
        
        
        for(int i = 0; i<enemies.size(); i++){
            
            Object* platform = enemies.at(i);
            float platformSize = platform->getSize();
            float3 posThat = float3(platform->getPosition().x,platform->getPosition().y, platform->getPosition().z);
            
            if (((position - posThat).norm())<platformSize){
                velocity.y = abs(velocity.y);
                platform->scale(float3(1.01,.99,1.01));
            }
        }
        float thresholda = 10;
        
        for(int i = 0; i<enemyBullets.size(); i++){
            
            float3 posBullet = enemyBullets.at(i)->getPosition();
            if (((position - posBullet).norm())<thresholda){
                //return true;
                return false;
            }
        }
        return false;
    }
    
    float getOrientation(){
        return orientationAngle;
    }
    
};


class SecondPlayer : public MeshInstance
{
    
    float maxVelocity= 500000;
    
    float3 velocity = float3(0, 0, 0);
    float angularVelocity = 0;
    float restitution = 1;
    float angularAcceleration = 0;
    float3 acceleration=float3(0,0,0);
    float cooldownRemaining=0;
    
    
public:
    SecondPlayer(Mesh* mesh, Material* material):MeshInstance(mesh, material){}
    
    virtual void move(double t, double dt){
        if (velocity.norm2()>maxVelocity) {
            velocity=velocity/velocity.norm()*maxVelocity;
        }
        else{
            velocity+= acceleration*dt;
            position+=velocity*dt;
            angularVelocity *= pow(0.1, dt);
            velocity *= pow(.7, dt);
            angularVelocity+=angularAcceleration*dt;
            orientationAngle += angularVelocity*dt;
            cooldownRemaining=cooldownRemaining-dt;
            if(position.y < 0) velocity.y *= -restitution;
        }
        
    }
    bool control(std::vector<bool>& keysPressed, std::vector<Object*>& spawn, std::vector<Object*>& objects, std::vector<Object*>& bullets, std::vector<LightSource*>& lightSources, std::vector<Object*>& platforms, std::vector<Object*>& enemies, std::vector<Object*>& enemyBullets){
        
        if(keysPressed.at('j'))
            angularAcceleration=200;
        else if(keysPressed.at('l'))
            angularAcceleration=-200;
        
        else
            angularAcceleration=0;
        
        if(keysPressed.at('i'))
            acceleration = float3(-cos((orientationAngle*M_PI)/180)*60, -10, sin((orientationAngle*M_PI)/180) *60);
        else if(keysPressed.at('k'))
            acceleration = float3(cos((orientationAngle*M_PI)/180) *60, -10, -sin((orientationAngle*M_PI)/180) *60);
        else
            acceleration=float3(0,-10,0);
        if(keysPressed['u'])
        {
            
            if(cooldownRemaining<=0){
                Bullet* bullet = new Bullet(getPosition(), float3(0, -5 ,0), new Material());
                spawn.push_back(bullet);
                cooldownRemaining=.5;
            }
        }
        if(keysPressed['n'])
        {
            
            if(cooldownRemaining<=0){
                Bullets* bullet = new Bullets(getPosition(), float3(cos(((orientationAngle-180)*M_PI)/180) * 60,-5,-sin(((orientationAngle-180)*M_PI)/180) *60), new Material());
                bullets.push_back(bullet);
                
                cooldownRemaining=.5;
                
            }
        }
        
        
        if(position.x>200){
            velocity.x=-velocity.x;
        }
        else if(position.x<-200){
            velocity.x=-velocity.x;
        }
        else if(position.z>200){
            velocity.z=-velocity.z;
        }
        else if(position.z<-200){
            velocity.z=-velocity.z;
        }
        
        
        float threshold = 15;
        
        
        for(int i = 0; i<spawn.size(); i++){
            
            float3 posThat = spawn.at(i)->getPosition();
            
            if (((position - posThat).norm())<threshold){
                acceleration.y = 20;
            }
        }
        
        
        
        for(int i = 0; i<platforms.size(); i++){
            
            Object* platform = platforms.at(i);
            float platformSize = platform->getSize();
            float3 posThat = float3(platform->getPosition().x,platform->getPosition().y, platform->getPosition().z);
            
            if (((position - posThat).norm())<platformSize){
                velocity.y = abs(velocity.y);
                
            }
        }
        
        
        for(int i = 0; i<enemies.size(); i++){
            
            Object* platform = enemies.at(i);
            float platformSize = platform->getSize();
            float3 posThat = float3(platform->getPosition().x,platform->getPosition().y, platform->getPosition().z);
            
            if (((position - posThat).norm())<platformSize){
                velocity.y = abs(velocity.y);
                platform->scale(float3(1.01,.99,1.01));
            }
        }
        float thresholda = 10;
        
        for(int i = 0; i<enemyBullets.size(); i++){
            
            float3 posBullet = enemyBullets.at(i)->getPosition();
            if (((position - posBullet).norm())<thresholda){
                return true;
                //return false;
            }
        }
        return false;
    }
    
    float getOrientation(){
        return orientationAngle;
    }
    
};



class TexturedMaterial :public Material
{
    unsigned int textureId;
    GLint filtering;
public:
TexturedMaterial(const char* filename,
                 GLint filtering = GL_LINEAR):filtering(filtering){
    unsigned char* data;
    int width;
    int height;
    int nComponents = 4;
    
    data = stbi_load(filename, &width, &height, &nComponents, 0);
    
    if(data == NULL) return;
    
    // opengl texture creation comes here
    
    delete data;
    
    glGenTextures(1, &textureId);  // id generation
    glBindTexture(GL_TEXTURE_2D, textureId);      // binding
    
    if(nComponents == 4)
        
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data); //uploading
        
        else if(nComponents == 3)
            
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data); //uploading

}
    
    void apply()
    {
//        first calls the superclass’ apply to set generic material properties
//        then enables opengl texturing
//        binds the texture that belongs to the material
//        sets filtering
//        GL_LINEAR means no mipmapping
//        GL_LINEAR_MIPMAP_LINEAR means mipmapping
        
        
        Material::apply();
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, textureId);
        glTexParameteri(GL_TEXTURE_2D,
                        GL_TEXTURE_MIN_FILTER,filtering);
        glTexParameteri(GL_TEXTURE_2D,
                        GL_TEXTURE_MAG_FILTER,filtering);
        glTexEnvi(GL_TEXTURE_ENV,
                        GL_TEXTURE_ENV_MODE, GL_MODULATE);

        
    }
    
};


class Scene
{
    
    float3 silverRI = float3(0.15,0.14,0.13);
    float3 silverEC = float3(3.7,3.11,2.47);
    Bouncer* avatar;
    float angle;
    Camera camera;
    std::vector<LightSource*> lightSources;
    std::vector<Object*> objects;
    std::vector<Object*> bullets;
    std::vector<Object*> enemyBullets;
    std::vector<Object*> spawn;
    std::vector<Material*> materials;
    std::vector<Object*> enemies;
    std::vector<Object*> platforms;
    std::vector<Mesh*> meshes;
public:
    void initialize()
    {
        // Scene is built here
        lightSources.push_back(
                               new DirectionalLight(
                                                    float3(0, 1, 0),
                                                    float3(.10,.20, .10)));
        lightSources.push_back(
                               new DirectionalLight(
                                                    float3(1, 0, 0),
                                                    float3(.20,.15, .20)));

        lightSources.push_back(
                               new DirectionalLight(
                                                    float3(1, 0, 1),
                                                    float3(.25,.20, .01)));
lightSources.push_back(
                               new PointLight(
                                              float3(10, 100, 0),
                                              float3(200.6, 10.2, 10.2)));
        
        lightSources.push_back(
                               new PointLight(
                                              float3(10, 200, 10),
                                              float3(200.6, 10.2, 10.2)));
        lightSources.push_back(
                               new PointLight(
                                              float3(10, 300, 10),
                                              float3(200.6, 10.2, 10.2)));
        

        
        
        
        lightSources.push_back(
                               new PointLight(
                                              float3(80, 50, 0),
                                              float3(1, 1, 0)));
        lightSources.push_back(
                               new PointLight(
                                              float3(20, 30, 18),
                                              float3(10, 10, 0)));
        

        
        
        //left lights
        
        lightSources.push_back(
                               new PointLight(
                                              float3(-199,  -1000, 199),
                                              float3(50.6, 3000.2, 3000.2)));
        lightSources.push_back(
                               new PointLight(
                                              float3(-199,  -500, 199),
                                              float3(500.6, 3000.2, 3000.2)));
        
        lightSources.push_back(
                               new PointLight(
                                              float3(-199,  0, 199),
                                              float3(500.6, 3000.2, 3000.2)));
        lightSources.push_back(
                               new PointLight(
                                              float3(-199,  500, 199),
                                              float3(500.6, 3000.2, 3000.2)));
        
        lightSources.push_back(
                               new PointLight(
                                              float3(-199,  1000, 199),
                                              float3(500.6, 3000.2, 3000.2)));
        
//right lights
        
        lightSources.push_back(
                               new PointLight(
                                              float3(199,  -1000, 199),
                                              float3(500.6, 3000.2, 3000.2)));
        lightSources.push_back(
                               new PointLight(
                                              float3(199,  -500, 199),
                                              float3(500.6, 3000.2, 3000.2)));
        
        lightSources.push_back(
                               new PointLight(
                                              float3(199,  0, 199),
                                              float3(500.6, 3000.2, 3000.2)));
        lightSources.push_back(
                               new PointLight(
                                              float3(199,  500, 199),
                                              float3(500.6, 3000.2, 3000.2)));
        
        lightSources.push_back(
                               new PointLight(
                                              float3(199,  1000, 199),
                                              float3(500.6, 3000.2, 3000.2)));
        
        

        
        
        
        Material* yellowDiffuseMaterial = new Material();
        materials.push_back(yellowDiffuseMaterial);
        yellowDiffuseMaterial->kd = float3(1, 1, 0);
        materials.push_back(new Material());
        materials.push_back(new Material());
        materials.push_back(new Material());
        materials.push_back(new Material());
        materials.push_back(new Material());
        materials.push_back(new Material());
        
        Mesh* mesh = new Mesh("/Users/joshuapitkofsky/Documents/Graphics/3D OpenGL/3D OpenGL/tigger.obj");
        Mesh* rocket = new Mesh("/Users/joshuapitkofsky/Documents/Graphics/3D OpenGL/3D OpenGL/tigger.obj");
        
        TexturedMaterial* rocketPaint = new TexturedMaterial("/Users/joshuapitkofsky/Documents/Graphics/3D OpenGL/3D OpenGL/tigger.png");
        
        TexturedMaterial* texturedMaterial = new TexturedMaterial("/Users/joshuapitkofsky/Documents/Graphics/3D OpenGL/3D OpenGL/tigger.png");

        TexturedMaterial* skyMaterial = new TexturedMaterial("/Users/joshuapitkofsky/Documents/Graphics/3D OpenGL/3D OpenGL/CutClouds.jpg");
        
        TexturedMaterial* groundTex = new TexturedMaterial("/Users/joshuapitkofsky/Documents/Graphics/3D OpenGL/3D OpenGL/ground.jpg");

        meshes.push_back(mesh);
        MeshInstance* meshInstance = new MeshInstance(mesh, texturedMaterial);
        Bouncer* bouncer = new Bouncer(mesh, texturedMaterial);
        bouncer->scale(float3(.07,.07,.07))->translate(float3(0,20,0));
        
        Ground* ground = new Ground(new Metal(silverRI,silverEC));
        Bouncer* ship = new Bouncer(rocket, rocketPaint);
        ship->scale(float3(400,400,400))->translate(float3(0,10,0));
        objects.push_back(ground);
        objects.push_back(new Sky(skyMaterial));
        avatar = bouncer;
       
        //objects.push_back(ship);
       objects.push_back(bouncer);
        
        if(twoPlayer){
            Bouncer* bouncer = new Bouncer(mesh, texturedMaterial);
            objects.push_back(bouncer);
            SecondPlayer* second= new SecondPlayer(mesh, texturedMaterial);
            objects.push_back(second);
        }
        
        Target* targeta = new Target(new Material());
        targeta->translate(float3(10,100,0));
        objects.push_back(targeta);
        
        Target* targetb = new Target(new Material());
        targetb->translate(float3(10,200,0));
        objects.push_back(targetb);
        
        Target* targetc = new Target(new Material());
        targetc->translate(float3(80,300,-20));
        objects.push_back(targetc);
        
        
        Platform* platforma = new Platform(new Material());
        platforma->translate(float3(80,50,0));
        platforms.push_back(platforma);
        
        Platform* platformb = new Platform(new Material());
        platformb->translate(float3(-80,190,0));
        platforms.push_back(platformb);
        
        Platform* platformc = new Platform(new Material());
        platformc->translate(float3(20,30,18));
        platforms.push_back(platformc);
        
        
        Enemy* enemya = new Enemy(new Material);
        enemya->translate(float3(20,70,18));
        enemies.push_back(enemya);
        
        Enemy* enemyb = new Enemy(new Material);
        enemyb->translate(float3(20,10,18));
        enemies.push_back(enemyb);
        
        Enemy* enemyc = new Enemy(new Material);
        enemyc->translate(float3(20,70,18));
        enemies.push_back(enemyc);
        
        Enemy* enemyd = new Enemy(new Material);
        enemyd->translate(float3(20,30,18));
        enemies.push_back(enemyd);
        
        Enemy* enemyf = new Enemy(new Material);
        enemyf->translate(float3(20,500,18));
        enemies.push_back(enemyf);
        
    }
    Bouncer* getAvatar(){
        return avatar;
    }
    
    
    ~Scene()
    {
        for (std::vector<LightSource*>::iterator iLightSource = lightSources.begin(); iLightSource != lightSources.end(); ++iLightSource)
            delete *iLightSource;
        for (std::vector<Material*>::iterator iMaterial = materials.begin(); iMaterial != materials.end(); ++iMaterial)
            delete *iMaterial;
        for (std::vector<Object*>::iterator iObject = objects.begin(); iObject != objects.end(); ++iObject)
            delete *iObject;
        for (std::vector<Object*>::iterator iObject = spawn.begin(); iObject != spawn .end(); ++iObject)
            delete *iObject;
        for (std::vector<Object*>::iterator iObject = bullets.begin(); iObject != bullets .end(); ++iObject)
            delete *iObject;
        for (std::vector<Object*>::iterator iObject = enemies.begin(); iObject != enemies .end(); ++iObject)
            delete *iObject;
        for (std::vector<Object*>::iterator iObject = enemyBullets.begin(); iObject != enemyBullets .end(); ++iObject)
            delete *iObject;


    }

    
public:
    Camera& getCamera()
    {
        return camera;
    }
    
    void move(float t, float dt){
        for (unsigned int iObject=0; iObject<objects.size(); iObject++)
            objects.at(iObject)->move(t,dt);
        for (unsigned int iObject=0; iObject<spawn.size(); iObject++)
            spawn.at(iObject)->move(t,dt);
        for (unsigned int iObject=0; iObject<bullets.size(); iObject++)
            bullets.at(iObject)->move(t,dt);
        for (unsigned int iObject=0; iObject<platforms.size(); iObject++)
            platforms.at(iObject)->move(t,dt);
        for (unsigned int iObject=0; iObject<enemies.size(); iObject++)
            enemies.at(iObject)->move(t,dt);
        for (unsigned int iObject=0; iObject<enemyBullets.size(); iObject++)
            enemyBullets.at(iObject)->move(t,dt);
    }
    
    void control(std::vector<bool>& keysPressed){
        


        for (std::vector<Object*>::iterator objectA = objects.begin(); objectA != objects.end(); ++objectA){
            if((*objectA)->control(keysPressed, spawn, objects, bullets, lightSources, platforms, enemies, enemyBullets)|| (*objectA)->flag){
            objects.erase(objectA);
            }
        }
        
        for (std::vector<Object*>::iterator objectA = spawn.begin(); objectA != spawn.end();){
            if((*objectA)->control(keysPressed, spawn, objects, bullets, lightSources, platforms, enemies, enemyBullets)){
                spawn.erase(objectA);
            }
            else{ ++objectA;}
        }
        
        for (std::vector<Object*>::iterator objectA = bullets.begin(); objectA != bullets.end();){
            if((*objectA)->control(keysPressed, spawn, objects, bullets, lightSources, platforms, enemies, enemyBullets)){
                bullets.erase(objectA);
            }
            else{ ++objectA;}
        }
        for (std::vector<Object*>::iterator objectA = enemies.begin(); objectA != enemies.end();){
            if((*objectA)->control(keysPressed, spawn, objects, bullets, lightSources, platforms, enemies, enemyBullets)){
                enemies.erase(objectA);
            }
            else{ ++objectA;}
        }
        for (std::vector<Object*>::iterator objectA = enemyBullets.begin(); objectA != enemyBullets.end();){
            if((*objectA)->control(keysPressed, spawn, objects, bullets, lightSources, platforms, enemies, enemyBullets)){
                enemyBullets.erase(objectA);
            }
            else{ ++objectA;}
        }

        for (std::vector<Object*>::iterator objectA = platforms.begin(); objectA != platforms.end();){
            if((*objectA)->control(keysPressed, spawn, objects, bullets, lightSources, platforms, enemies, enemyBullets)){
                platforms.erase(objectA);
            }
            else{ ++objectA;}
        }

     }


    void draw()

    {
        float3 lightDir = lightSources.at(0)->getLightDirAt(float3(0, 0, 0));

        camera.apply();
        unsigned int iLightSource=0;
        for (; iLightSource<lightSources.size(); iLightSource++)
        {
            glEnable(GL_LIGHT0 + iLightSource);
            lightSources.at(iLightSource)->apply(GL_LIGHT0 + iLightSource);
        }
        for (; iLightSource<GL_MAX_LIGHTS; iLightSource++)
            glDisable(GL_LIGHT0 + iLightSource);
        
        for (unsigned int iObject=0; iObject<objects.size(); iObject++)
            objects.at(iObject)->draw();
        
        for (unsigned int iObject=0; iObject<spawn.size(); iObject++)
            spawn.at(iObject)->draw();
        for (unsigned int iObject=0; iObject<bullets.size(); iObject++)
            bullets.at(iObject)->draw();
        
        for (unsigned int iObject=0; iObject<platforms.size(); iObject++)
            platforms.at(iObject)->draw();
        
        for (unsigned int iObject=0; iObject<enemies.size(); iObject++)
            enemies.at(iObject)->draw();
        for (unsigned int iObject=0; iObject<enemyBullets.size(); iObject++)
            enemyBullets.at(iObject)->draw();
        
        glDisable(GL_LIGHTING);
        glDisable(GL_TEXTURE_2D);
        glColor3d(0, 0, 0);
        for (unsigned int iObject=0; iObject<objects.size(); iObject++)
            objects.at(iObject)->drawShadow(float3(0,10,0));
        for (unsigned int iObject=0; iObject<spawn.size(); iObject++)
            spawn.at(iObject)->drawShadow(float3(0,10,0));
        for (unsigned int iObject=0; iObject<bullets.size(); iObject++)
            bullets.at(iObject)->drawShadow(float3(0,10,0));
        for (unsigned int iObject=0; iObject<platforms.size(); iObject++)
            platforms.at(iObject)->drawShadow(float3(0,10,0));
        for (unsigned int iObject=0; iObject<enemies.size(); iObject++)
            enemies.at(iObject)->drawShadow(float3(0,10,0));
        for (unsigned int iObject=0; iObject<enemyBullets.size(); iObject++)
            enemyBullets.at(iObject)->drawShadow(float3(0,10,0));
        
        glEnable(GL_LIGHTING);
        glEnable(GL_TEXTURE_2D);
    }
};

Scene scene;
std::vector<bool> keysPressed;

void onDisplay( ) {
    glClearColor(0.1f, 0.2f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear screen
    
    scene.draw();
    
    glutSwapBuffers(); // drawing finished
}

void onIdle()
{
    double t = glutGet(GLUT_ELAPSED_TIME) * 0.001;        	// time elapsed since starting this program in msec
    static double lastTime = 0.0;
    double dt = t - lastTime;
    lastTime = t;
    Bouncer* avatar = scene.getAvatar();
    scene.control(keysPressed);
    scene.move(t,dt);
    float3 pos = scene.getAvatar()->getPosition();
    float orientation = avatar->getOrientation();
    

    scene.getCamera().move(dt, keysPressed, pos, orientation);
    glutPostRedisplay();
}

void onKeyboard(unsigned char key, int x, int y)
{
    keysPressed.at(key) = true;
}

void onKeyboardUp(unsigned char key, int x, int y)
{
    keysPressed.at(key) = false;
}

void onMouse(int button, int state, int x, int y)
{
    if(button == GLUT_LEFT_BUTTON)
        if(state == GLUT_DOWN)
            scene.getCamera().startDrag(x, y);
        else
            scene.getCamera().endDrag();
}

void onMouseMotion(int x, int y)
{
    scene.getCamera().drag(x, y);
}

void onReshape(int winWidth, int winHeight)
{
    glViewport(0, 0, winWidth, winHeight);
    scene.getCamera().setAspectRatio((float)winWidth/winHeight);
}	

int main(int argc, char **argv) {
    glutInit(&argc, argv);						// initialize GLUT
    glutInitWindowSize(600, 600);				// startup window size 
    glutInitWindowPosition(100, 100);           // where to put window on screen
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);    // 8 bit R,G,B,A + double buffer + depth buffer
    
    glutCreateWindow("OpenGL teapots");				// application window is created and displayed
    
    glViewport(0, 0, 600, 600);
    
    glutDisplayFunc(onDisplay);					// register callback
    glutIdleFunc(onIdle);						// register callback
    glutReshapeFunc(onReshape);
    glutKeyboardFunc(onKeyboard);
    glutKeyboardUpFunc(onKeyboardUp);
    glutMouseFunc(onMouse);
    glutMotionFunc(onMouseMotion);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    
    scene.initialize();
    for(int i=0; i<256; i++)
        keysPressed.push_back(false);
    
    glutMainLoop();								// launch event handling loop
    
    return 0;
}
