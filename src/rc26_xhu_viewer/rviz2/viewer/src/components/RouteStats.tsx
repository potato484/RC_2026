export function RouteStats({
  title,
  value,
}: {
  title: string;
  value: string;
}) {
  return (
    <div className="stat-card">
      <span className="stat-label">{title}</span>
      <span className="stat-value">{value}</span>
    </div>
  );
}
