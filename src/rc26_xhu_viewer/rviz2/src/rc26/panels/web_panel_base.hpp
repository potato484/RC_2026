#ifndef RC26_XHU_VIEWER__PANELS__WEB_PANEL_BASE_HPP_
#define RC26_XHU_VIEWER__PANELS__WEB_PANEL_BASE_HPP_

#include <rviz_common/panel.hpp>

class QLabel;

#ifdef HAS_QT_WEBENGINE
class QWebEngineView;
#endif

namespace rc26_xhu_viewer
{

class WebPanelBase : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WebPanelBase(const QString & html_resource, QWidget * parent = nullptr);
  bool webAvailable() const;

protected:
#ifdef HAS_QT_WEBENGINE
  QWebEngineView * web_view_{nullptr};
  void loadHtml(const QString & path);
  void runJavaScript(const QString & js);
#endif

private:
  QLabel * fallback_label_{nullptr};
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__WEB_PANEL_BASE_HPP_
