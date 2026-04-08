#include "web_panel_base.hpp"

#include <QLabel>
#include <QVBoxLayout>

#ifdef HAS_QT_WEBENGINE
#include <QWebEngineView>
#include <QUrl>
#include <QFile>
#endif

namespace rc26_xhu_viewer
{

WebPanelBase::WebPanelBase(const QString & html_resource, QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

#ifdef HAS_QT_WEBENGINE
  web_view_ = new QWebEngineView(this);
  layout->addWidget(web_view_);
  loadHtml(html_resource);
#else
  (void)html_resource;
  fallback_label_ = new QLabel(
    QStringLiteral("Web \u9762\u677f\u4e0d\u53ef\u7528\uff1a\u7f3a\u5c11 Qt5WebEngine \u4f9d\u8d56"));
  fallback_label_->setAlignment(Qt::AlignCenter);
  fallback_label_->setStyleSheet("color: #999; font-size: 14px;");
  layout->addWidget(fallback_label_);
#endif
}

bool WebPanelBase::webAvailable() const
{
#ifdef HAS_QT_WEBENGINE
  return true;
#else
  return false;
#endif
}

#ifdef HAS_QT_WEBENGINE
void WebPanelBase::loadHtml(const QString & path)
{
  if (QFile::exists(path)) {
    web_view_->load(QUrl::fromLocalFile(path));
  }
}

void WebPanelBase::runJavaScript(const QString & js)
{
  web_view_->page()->runJavaScript(js);
}
#endif

}  // namespace rc26_xhu_viewer
