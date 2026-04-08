#include "rc26_xhu_viewer_frame.hpp"

#include <QAction>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QActionGroup>

#include "rviz_common/load_resource.hpp"
#include "rviz_common/visualization_manager.hpp"

#include "rc26_display_factory.hpp"

namespace rc26_xhu_viewer
{

RC26XhuViewerFrame::RC26XhuViewerFrame(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
  QWidget * parent)
: rviz_common::VisualizationFrame(rviz_ros_node, parent)
{
}

void RC26XhuViewerFrame::initialize(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
  const QString & display_config_file)
{
  VisualizationFrame::initialize(rviz_ros_node, display_config_file);
  injectDisplayFactory();
}

void RC26XhuViewerFrame::injectDisplayFactory()
{
  if (manager_) {
    manager_->setDisplayFactory(new RC26DisplayFactory());
  }
}

void RC26XhuViewerFrame::initMenus()
{
  file_menu_ = menuBar()->addMenu(QStringLiteral("文件(&F)"));

  QAction * open_action = file_menu_->addAction(
    QStringLiteral("打开配置(&O)"), this, SLOT(onOpen()), QKeySequence("Ctrl+O"));
  this->addAction(open_action);

  QAction * save_action = file_menu_->addAction(
    QStringLiteral("保存配置(&S)"), this, SLOT(onSave()), QKeySequence("Ctrl+S"));
  this->addAction(save_action);

  QAction * save_as_action = file_menu_->addAction(
    QStringLiteral("另存配置(&A)"), this, SLOT(onSaveAs()), QKeySequence("Ctrl+Shift+S"));
  this->addAction(save_as_action);

  recent_configs_menu_ = file_menu_->addMenu(QStringLiteral("最近配置(&R)"));
  file_menu_->addAction(QStringLiteral("保存截图(&I)"), this, SLOT(onSaveImage()));
  file_menu_->addSeparator();

  QAction * quit_action = file_menu_->addAction(
    QStringLiteral("退出(&Q)"), this, SLOT(close()), QKeySequence("Ctrl+Q"));
  this->addAction(quit_action);

  view_menu_ = menuBar()->addMenu(QStringLiteral("面板(&P)"));
  view_menu_->addAction(QStringLiteral("添加面板(&N)"), this, SLOT(openNewPanelDialog()));
  delete_view_menu_ = view_menu_->addMenu(QStringLiteral("删除面板(&D)"));
  delete_view_menu_->setEnabled(false);

  QAction * fullscreen_action = view_menu_->addAction(
    QStringLiteral("全屏(&F)"), this, SLOT(setFullScreen(bool)), Qt::Key_F11);
  fullscreen_action->setCheckable(true);
  this->addAction(fullscreen_action);
  connect(this, SIGNAL(fullScreenChange(bool)), fullscreen_action, SLOT(setChecked(bool)));
  view_menu_->addSeparator();

  QMenu * help_menu = menuBar()->addMenu(QStringLiteral("帮助(&H)"));
  help_menu->addAction(QStringLiteral("关于(&A)"), this, SLOT(onHelpAbout()));
}

void RC26XhuViewerFrame::initToolbars()
{
  QFont font;
  font.setPointSize(font.pointSizeF() * 0.9);

  toolbar_ = addToolBar(QStringLiteral("工具"));
  toolbar_->setFont(font);
  toolbar_->setContentsMargins(0, 0, 0, 0);
  toolbar_->setObjectName("Tools");
  toolbar_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);

  toolbar_actions_ = new QActionGroup(this);
  connect(
    toolbar_actions_, SIGNAL(triggered(QAction *)), this,
    SLOT(onToolbarActionTriggered(QAction *)));

  view_menu_->addAction(toolbar_->toggleViewAction());
}

}  // namespace rc26_xhu_viewer
