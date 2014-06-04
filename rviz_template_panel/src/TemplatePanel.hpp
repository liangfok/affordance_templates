#ifndef TEMPLATE_PANEL_HPP
#define TEMPLATE_PANEL_HPP

#include "Affordance.hpp"

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <rviz/panel.h>
#include <rviz/selection/forwards.h>
#include <rviz/selection/selection_handler.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/default_plugin/markers/marker_selection_handler.h>

// qt
#include <QGridLayout>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>

// zmq
#include <zmq.hpp>
#include "AffordanceTemplateServerCmd.pb.h"

using namespace std;

namespace rviz_template_panel
{
    class TemplatePanel : public rviz::Panel
    {
    Q_OBJECT
    public:
        typedef boost::shared_ptr<Affordance> AffordanceSharedPtr;

        TemplatePanel(QWidget* parent=0);
        ~TemplatePanel();

    public Q_SLOTS:
        void addTemplate();

        /** \brief Send a ZMQ request to get available template classes and populate the template list.
         */
        void getAvailableTemplates();

        /** \brief Send a ZMQ request to get running templates on the server.
         */
        void getRunningTemplates();

        /** \brief Connect to the template server.
         */
        void connect();

        /** \brief Disconnect from the template server.
         */
        void disconnect();

        /** \brief Send a ZMQ request to kill a running template.
         */
        void killTemplate(QListWidgetItem* item);

    private:
        void setupWidgets();

        void removeTemplates();

        void sendAdd(const string& class_name);
        void sendKill(const string& class_name, int id);
        void sendPing();
        void sendShutdown();

        // TODO: template these template functions that keep track of templates
        bool addAffordance(const AffordanceSharedPtr& obj);
        bool removeAffordance(const AffordanceSharedPtr& obj);
        bool checkAffordance(const AffordanceSharedPtr& obj);
        void send_request(const Request& request, Response& response);

        // GUI Widgets
        QGridLayout* gridLayout;
        QPushButton* connectButton;
        QPushButton* disconnectButton;
        QLabel* serverLabel;
        QLabel* statusLabel;
        QGraphicsView* graphicsView;
        QGraphicsScene* graphicsScene;
        QListWidget* runningList;

        // map to track instantiated object templates
        std::map<std::string, AffordanceSharedPtr> affordanceMap;

        // zmq
        zmq::context_t context;
        zmq::socket_t* socket;
        bool connected;
    };
}

#endif
