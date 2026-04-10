#include "web_bridge.hpp"

namespace rc26_xhu_viewer
{

WebBridge::WebBridge(QObject * parent)
: QObject(parent)
{
}

QString WebBridge::lastData() const
{
  return last_data_;
}

void WebBridge::pushData(const QString & json)
{
  last_data_ = json;
  Q_EMIT dataChanged();
}

}  // namespace rc26_xhu_viewer
