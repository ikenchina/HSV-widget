/****************************************************************************
**
** Copyright (C) 2013 Ken
** All rights reserved.
** Contact: ikenchina@gmail.com
**
****************************************************************************/
#ifndef __KCOLORCIRCLEHSV_H__
#define __KCOLORCIRCLEHSV_H__
#include <QtGui/QImage>
#include <QtGui/QWidget>




class  KColorCircleHsv : public QWidget
{
	Q_OBJECT

public:
	KColorCircleHsv(QWidget *parent = 0);
	~KColorCircleHsv();
	QColor color() const;

signals:
	void colorChanged(const QColor &col);

public slots:
	void setColor(qreal h, qreal s, qreal l);
	void setColor(const QColor &col);
	
protected:
	void paintEvent(QPaintEvent *);
	void mouseMoveEvent(QMouseEvent *);
	void mousePressEvent(QMouseEvent *);
	void mouseReleaseEvent(QMouseEvent *);
	void keyPressEvent(QKeyEvent *e);
	void resizeEvent(QResizeEvent *);
	
private:
	// double型color
	// 三角形渐变中：SV的差度/y差度=SV增量比, 所以用SV的渐变需要用qreal,避免累计中精度丢失
	struct DoubleColor
	{
		qreal r, g, b;
		DoubleColor() : r(0.0), g(0.0), b(0.0) {}
		DoubleColor(qreal red, qreal green, qreal blue) : r(red), g(green), b(blue) {}
	};
	
	// 三角形三个顶点
	struct Vertex
	{
		DoubleColor color;
		QPointF point;
		
		Vertex(const QColor &c, const QPointF &p)
		: color(DoubleColor((qreal) c.red(), (qreal) c.green(),
					(qreal) c.blue())), point(p) {}
		bool operator<(const Vertex &other) const
		{
			return point.y() < other.point.y();
		}
	};
	
	void calVertexPoint();
	void calRadian(int hue);
	bool pointChanged(QPointF point);
	
	QPointF pointFromColor(const QColor &col) const;
	QColor colorFromPoint(const QPointF &p) const;
	
	void createBackground();
	void paintImage();
	void drawTriangle(QImage *p, const QPointF &a, const QPointF &b,
					const QPointF &c, const QColor &color);
	
	QImage m_imgBG;
	QImage m_buf;
	
	double m_radA, m_radB, m_radC;
	QPointF pa, pb, pc, pd;
	QColor m_OldColor;
	QColor m_CurrentColor;
	int m_nCurrentHue;
	
	bool m_bNeedUpdateBackground;
	int m_nPenWidth;
	
	int m_nSVEllipseSize;
	int m_nOuterRadius;
	double m_dOuterInnerWidth;
	
	QPointF m_dSelectorPos;
	
	enum ESelectMode
	{
		None,
		SelCircle,
		SelTriangle
	}m_selMode;
	
};

#endif  //__KCOLORCIRCLEHSV_H__
