#include "rc26_xhu_viewer_frame.hpp"

#include <QAction>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QToolButton>
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

void RC26XhuViewerFrame::configureVisualizationManager()
{
  if (manager_) {
    manager_->setDisplayFactory(new RC26DisplayFactory());
  }
}

void RC26XhuViewerFrame::initMenus()
{
  file_menu_ = menuBar()->addMenu(QStringLiteral("文件"));

  QAction * open_action = file_menu_->addAction(
    QStringLiteral("打开配置"), this, SLOT(onOpen()), QKeySequence("Ctrl+O"));
  this->addAction(open_action);

  QAction * save_action = file_menu_->addAction(
    QStringLiteral("保存配置"), this, SLOT(onSave()), QKeySequence("Ctrl+S"));
  this->addAction(save_action);

  QAction * save_as_action = file_menu_->addAction(
    QStringLiteral("另存配置"), this, SLOT(onSaveAs()), QKeySequence("Ctrl+Shift+S"));
  this->addAction(save_as_action);

  recent_configs_menu_ = file_menu_->addMenu(QStringLiteral("最近配置"));
  file_menu_->addAction(QStringLiteral("保存截图"), this, SLOT(onSaveImage()));
  file_menu_->addSeparator();

  QAction * quit_action = file_menu_->addAction(
    QStringLiteral("退出"), this, SLOT(close()), QKeySequence("Ctrl+Q"));
  this->addAction(quit_action);

  view_menu_ = menuBar()->addMenu(QStringLiteral("面板"));
  view_menu_->addAction(QStringLiteral("添加面板"), this, SLOT(openNewPanelDialog()));
  delete_view_menu_ = view_menu_->addMenu(QStringLiteral("删除面板"));
  delete_view_menu_->setEnabled(false);

  QAction * fullscreen_action = view_menu_->addAction(
    QStringLiteral("全屏"), this, SLOT(setFullScreen(bool)), Qt::Key_F11);
  fullscreen_action->setCheckable(true);
  this->addAction(fullscreen_action);
  connect(this, SIGNAL(fullScreenChange(bool)), fullscreen_action, SLOT(setChecked(bool)));
  view_menu_->addSeparator();

  QMenu * help_menu = menuBar()->addMenu(QStringLiteral("帮助"));
  help_menu->addAction(QStringLiteral("关于"), this, SLOT(onHelpAbout()));
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

  add_tool_action_ = new QAction(QString(), toolbar_actions_);
  add_tool_action_->setToolTip(QStringLiteral("添加工具"));
  add_tool_action_->setIcon(rviz_common::loadPixmap("package://rviz_common/icons/plus.png"));
  toolbar_->addAction(add_tool_action_);
  connect(add_tool_action_, SIGNAL(triggered()), this, SLOT(openNewToolDialog()));

  remove_tool_menu_ = new QMenu(this);
  auto * remove_tool_button = new QToolButton(this);
  remove_tool_button->setMenu(remove_tool_menu_);
  remove_tool_button->setPopupMode(QToolButton::InstantPopup);
  remove_tool_button->setToolTip(QStringLiteral("移除工具"));
  remove_tool_button->setIcon(rviz_common::loadPixmap("package://rviz_common/icons/minus.png"));
  toolbar_->addWidget(remove_tool_button);
  connect(
    remove_tool_menu_, SIGNAL(triggered(QAction *)), this,
    SLOT(onToolbarRemoveTool(QAction *)));
}

}  // namespace rc26_xhu_viewer
