#include "TemplatePanel.hpp"

// TODO: don't like these
#define PIXMAP_SIZE 100
#define XOFFSET 20
#define YOFFSET 20
#define CLASS_INDEX 0

using namespace rviz_template_panel;
using namespace std;

/** \brief Helper function to return a new socket.
 */
static zmq::socket_t* client_socket (zmq::context_t& context) {
    zmq::socket_t* client = new zmq::socket_t (context, ZMQ_REQ);
    client->connect("tcp://localhost:6789");

    // configure socket to not wait at close time
    int linger = 0;
    client->setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
    return client;
}

static vector<string> &split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

/** \brief Split a string by a delimiter and return it in a vector.
 */
static vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

TemplatePanel::TemplatePanel(QWidget* parent)
    : rviz::Panel(parent),
      context(1),
      connected(false) {
    setupWidgets();
    connect();
}

TemplatePanel::~TemplatePanel() {
    delete gridLayout;
    delete connectButton;
    delete disconnectButton;
    delete serverLabel;
    delete statusLabel;
    delete graphicsView;
    delete graphicsScene;
    delete socket;
}

void TemplatePanel::setupWidgets() {
    gridLayout = new QGridLayout(this);
    serverLabel = new QLabel("localhost:6789");
    serverLabel->setAlignment(Qt::AlignCenter);
    statusLabel = new QLabel("Disconnected");
    statusLabel->setAlignment(Qt::AlignCenter);
    statusLabel->setStyleSheet("QLabel {color: red;}");

    connectButton = new QPushButton("Connect");
    disconnectButton = new QPushButton("Disconnect");

    runningList = new QListWidget();
    runningList->setMinimumHeight(200);
    QObject::connect(runningList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(killTemplate(QListWidgetItem*)));

    graphicsScene = new QGraphicsScene(this);
    graphicsView = new QGraphicsView(graphicsScene, this);

    QObject::connect(connectButton, SIGNAL(clicked()), this, SLOT(connect()));
    QObject::connect(disconnectButton, SIGNAL(clicked()), this, SLOT(disconnect()));

    gridLayout->addWidget(serverLabel, 0, 0, 1, -1);
    gridLayout->addWidget(statusLabel, 1, 0, 1, -1);
    gridLayout->addWidget(connectButton, 2, 0, 1, 1);
    gridLayout->addWidget(disconnectButton, 2, 1, 1, 1);
    gridLayout->addWidget(runningList, 3, -1, 1, 1);
    gridLayout->addWidget(graphicsView, 4, 0, 1, -1);

    setLayout(gridLayout);
}

void TemplatePanel::getAvailableTemplates() {
    if (connected) {
        // templates must be exported in plugin_description.xml of marker_template package
        Request req;
        req.set_type(Request::QUERY);
        Response rep;
        send_request(req, rep);
        int yoffset = YOFFSET;

        string package_path = ros::package::getPath("template_markers");
        for (auto& c: rep.marker_template()) {

            AffordanceSharedPtr pitem(new Affordance(c.type(), c.image_path()));

            pitem->setPos(XOFFSET, yoffset);
            yoffset += PIXMAP_SIZE + YOFFSET;

            graphicsScene->addItem(pitem.get());
            addAffordance(pitem);
        }
        QObject::connect(graphicsScene, SIGNAL(selectionChanged()), this, SLOT(addTemplate()));
        graphicsScene->update();
    }
}

void TemplatePanel::removeTemplates() {
    graphicsScene->disconnect(SIGNAL(selectionChanged()));
    for (auto& pitem: graphicsScene->items()) {
        graphicsScene->removeItem(pitem);
    }
    affordanceMap.clear();
    graphicsScene->update();
}

void TemplatePanel::sendAdd(const string& class_name) {
    Request req;
    req.set_type(Request::ADD);
    Template* temp(req.add_marker_template());
    temp->set_type(class_name);
    Response resp;
    send_request(req, resp);
}

void TemplatePanel::sendKill(const string& class_name, int id) {
    Request req;
    req.set_type(Request::KILL);
    Template* temp(req.add_marker_template());
    temp->set_type(class_name);
    temp->set_id(id);
    Response resp;
    send_request(req, resp);
}

void TemplatePanel::killTemplate(QListWidgetItem* item) {
    vector<string> template_info = split(item->text().toUtf8().constData(), ':');
    int id;
    istringstream(template_info[1]) >> id;
    sendKill(template_info[0], id);
    getRunningTemplates();
}

void TemplatePanel::sendShutdown() {
    Request req;
    req.set_type(Request::SHUTDOWN);
    Response resp;
    send_request(req, resp);
}

void TemplatePanel::sendPing() {
    Request req;
    req.set_type(Request::PING);
    Response resp;
    send_request(req, resp);
}

void TemplatePanel::getRunningTemplates() {
    Request req;
    req.set_type(Request::RUNNING);
    Response resp;
    send_request(req, resp);

    runningList->clear();

    for (auto& c: resp.marker_template()) {
        string name = c.type() + ":" + to_string(c.id());
        runningList->addItem(QString::fromStdString(name));
    }
    runningList->sortItems();
}

void TemplatePanel::connect() {
    // return if already connected
    if (connected) {
        return;
    }

    try {
        socket = client_socket(context);
        connected = true;
        statusLabel->setText("Connected");
        statusLabel->setStyleSheet("QLabel {color: green;}");
        sendPing();
        getAvailableTemplates();
        getRunningTemplates();
    } catch (const zmq::error_t& ex) {
        cerr << ex.what() << endl;
    }
}

void TemplatePanel::disconnect() {
    // return if already disconnected
    if (!connected) {
        return;
    }

    try {
        runningList->clear();
        removeTemplates();
        socket->close();
        connected = false;
        statusLabel->setText("Disconnected");
        statusLabel->setStyleSheet("QLabel {color: red;}");
        delete socket;
    } catch (const zmq::error_t& ex) {
        cerr << ex.what() << endl;
    }
}

void TemplatePanel::addTemplate() {
    // Add an object template to the InteractiveMarkerServer for each selected item.
    QList<QGraphicsItem*> list = graphicsScene->selectedItems();
    for (int i=0; i < list.size(); ++i) {
        // Get the object template class name from the first element in the QGraphicsItem's custom data
        // field. This field is set in the derived Affordance class when setting up the widgets.
        string class_name = list.at(i)->data(CLASS_INDEX).toString().toStdString();
        sendAdd(class_name);
    }

    // update running templates
    getRunningTemplates();
}

void TemplatePanel::send_request(const Request& request, Response& response) {
    if (connected) {
        try {
            string req;
            request.SerializeToString(&req);

            zmq::message_t msg(req.size());
            memcpy((void*) msg.data(), req.data(), req.size());
            socket->send(msg);

            string rep;
            zmq::pollitem_t poller[] = { {*socket, 0, ZMQ_POLLIN, 0} };
            zmq::poll(&poller[0], 1, 1000000);

            // poll for 1 second
            if (poller[0].revents & ZMQ_POLLIN) {

                zmq::message_t reply;
                socket->recv(&reply);

                response.ParseFromArray(reply.data(), reply.size());

            } else {
                disconnect();
            }
        } catch (const zmq::error_t& ex) {
            cerr << ex.what() << endl;
        }
    }
}

bool TemplatePanel::addAffordance(const AffordanceSharedPtr& obj) {
    // check if template is in our map
    if (!checkAffordance(obj)) {
        affordanceMap[(*obj).key()] = obj;
        return true;
    }
    return false;
}

bool TemplatePanel::removeAffordance(const AffordanceSharedPtr& obj) {
    // check if template is in our map
    if (checkAffordance(obj)) {
        affordanceMap.erase((*obj).key());
        return true;
    }
    return false;
}

bool TemplatePanel::checkAffordance(const AffordanceSharedPtr& obj) {
    if (affordanceMap.find((*obj).key()) == affordanceMap.end()) {
        return false;
    }
    return true;
}

#include <pluginlib/class_list_macros.h>
#if ROS_VERSION_MINIMUM(1,9,41)
    // starting with Groovy
    // PLUGINLIB_EXPORT_CLASS(class_type, base_class_type)
    PLUGINLIB_EXPORT_CLASS(rviz_template_panel::TemplatePanel, rviz::Panel)
#else
    // PLUGINLIB_DECLARE_CLASS(pkg, class_name, class_type, base_class_type)
    PLUGINLIB_DECLARE_CLASS(rviz_template_panel, TemplatePanel, rviz_template_panel::TemplatePanel, rviz::Panel)
#endif
