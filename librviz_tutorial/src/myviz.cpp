/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QDebug>
#include <QPushButton>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/tool_manager.h"
#include "myviz.h"


// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : QWidget( parent )
{
  // Construct and lay out labels and slider controls.
  QLabel* thickness_label = new QLabel( "Line Thickness" );
  QSlider* thickness_slider = new QSlider( Qt::Horizontal );
  thickness_slider->setMinimum( 1 );
  thickness_slider->setMaximum( 100 );
  QLabel* cell_size_label = new QLabel( "Cell Size" );
  QSlider* cell_size_slider = new QSlider( Qt::Horizontal );
  cell_size_slider->setMinimum( 1 );
  cell_size_slider->setMaximum( 100 );
  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( thickness_label, 0, 0 );
  controls_layout->addWidget( thickness_slider, 0, 1 );
  controls_layout->addWidget( cell_size_label, 1, 0 );
  controls_layout->addWidget( cell_size_slider, 1, 1 );

  QPushButton* select = new QPushButton( "Select" );
  QPushButton* navGoal = new QPushButton( "2D Nav Goal" );
  QPushButton* poseEstimate = new QPushButton( "2D Pose Estimate" );

  //QAction action;
  //button->addAction();

  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( controls_layout );
  main_layout->addWidget(select);
  main_layout->addWidget(navGoal);
  main_layout->addWidget(poseEstimate);
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Make signal/slot connections.
  connect( thickness_slider, SIGNAL( valueChanged( int )), this, SLOT( setThickness( int )));
  connect( cell_size_slider, SIGNAL( valueChanged( int )), this, SLOT( setCellSize( int )));
  connect( select, SIGNAL(clicked()), this, SLOT( onClickSelect()));
  connect( navGoal, SIGNAL(clicked()), this, SLOT( onClickNavGoal()));
  connect( poseEstimate, SIGNAL(clicked()), this, SLOT( onClickPoseEstimate()));

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->initialize();
  manager_->startUpdate();
  // Create a Grid display.

manager_->setFixedFrame("map");

robotModel_ = manager_->createDisplay( "rviz/RobotModel", "robot model", true );
grid_       = manager_->createDisplay( "rviz/Grid", "grid", true );
poseAraay_  = manager_->createDisplay( "rviz/PoseArray", "pose array", true );
tF_         = manager_->createDisplay( "rviz/TF","TF", true );
laserScan_  = manager_->createDisplay( "rviz/LaserScan","Laser", true );
map1_       = manager_->createDisplay( "rviz/Map","Map1", true );
polygon_    = manager_->createDisplay( "rviz/Polygon","Polygon", true );
path1_      = manager_->createDisplay( "rviz/Path","Path1", true );
path2_      = manager_->createDisplay( "rviz/Path","Path2", true );
marker_     = manager_->createDisplay( "rviz/Marker","Marker", true );
map2_       = manager_->createDisplay( "rviz/Map","Map2", true );
//map3_     = manager_->createDisplay( "rviz/Map","Map3", true );
pointCloud_ = manager_->createDisplay( "rviz/PointCloud2","PointCloud", true );

ROS_ASSERT( grid_ != NULL );


  // Configure the GridDisplay the way we like it.
  grid_->subProp( "Line Style" )->setValue( "Billboards" );
  grid_->subProp( "Color" )->setValue( Qt::gray );

  poseAraay_->subProp( "Topic" )->setValue("/particlecloud");
  poseAraay_->subProp( "Color" )->setValue(Qt::red);
  poseAraay_->subProp( "Arrow Length" )->setValue(0.3);

  tF_->subProp( "Show Names" )->setValue(true);
  tF_->subProp( "Show Axes" )->setValue(true);
  tF_->subProp( "Show Arrows" )->setValue(true);
  tF_->subProp( "Marker Scale" )->setValue(1);
  tF_->subProp( "Frame Timeout" )->setValue(15);

  laserScan_->subProp( "Topic" )->setValue("/scan");

  map1_->subProp( "Topic" )->setValue("/map");

  polygon_->subProp( "Topic" )->setValue("/move_base/local_costmap/obstacle_layer_footprint/footprint_stamped");
  polygon_->subProp("Color")->setValue(Qt::green);

  path1_->subProp( "Topic" )->setValue("/move_base/TrajectoryPlannerROS/global_plan");
  path1_->subProp("Color")->setValue(Qt::green);

  path2_->subProp( "Topic" )->setValue("/move_base/NavfnROS/plan");
  path2_->subProp("Color")->setValue(Qt::yellow);


  marker_->subProp( "Topic" )->setValue("/exploration_polygon_marker");

  map2_->subProp( "Topic" )->setValue("/explore_server/explore_costmap/costmap");

  //map3_->subProp( "Topic" )->setValue("/explore_server/explore_costmap/costmap");//costmap for navigation stack

  pointCloud_->subProp( "Topic" )->setValue("/camera/depth_registered/points");

  //camera
 // grid2_->subProp( "Image Topic" )->setValue("/camera/rgb/image_raw");

  // Initialize the slider values.
  thickness_slider->setValue( 25 );
  cell_size_slider->setValue( 10 );
}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the line thickness of the grid by changing the
// grid's "Line Width" property.
void MyViz::setThickness( int thickness_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( thickness_percent / 100.0f );
  }
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the cell size of the grid by changing the grid's
// "Cell Size" Property.
void MyViz::setCellSize( int cell_size_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Cell Size" )->setValue( cell_size_percent / 10.0f );
  }
}


void MyViz::onClickNavGoal()
{
  if( grid_ != NULL )
  {
       navGoal_ = manager_->getToolManager();

      for(int i=0; i<navGoal_->numTools(); i++)
      {
          if(navGoal_->getTool(i)->getName() == "2D Nav Goal")
          {
              navGoal_->setCurrentTool(navGoal_->getTool(i));
          }
      }
}

}
  void MyViz::onClickPoseEstimate()
  {
      if( grid_ != NULL )
      {
         poseEstimate_ = manager_->getToolManager();

        for(int i=0; i<poseEstimate_->numTools(); i++)
        {
            if(poseEstimate_->getTool(i)->getName() == "2D Pose Estimate")
            {
                poseEstimate_->setCurrentTool(poseEstimate_->getTool(i));
            }
        }
      }
  }
    void MyViz::onClickSelect()
    {
           select_ = manager_->getToolManager();

          for(int i=0; i<select_->numTools(); i++)
          {
              if(select_->getTool(i)->getName() == "Select")
              {
                  select_->setCurrentTool(select_->getTool(i));
              }
          }

    }





