

#include "kcolorcirclehsv.h"
#include <math>


#define HSVPI 3.1415926535897932
#define HSVTWOPI (2.0*HSVPI)


// ***************** 几何算法 geometry algorithms

// 勾股定理
// Pythagoreans theorem
qreal pythagorean(qreal l1, qreal l2, bool sub = false)
{
	if (sub)
		return sqrt(l1 * l1 - l2 * l2);
	return sqrt(l1 * l1 + l2 * l2);
}


/* 点积
 * 矢量(p1-op)和(p2-op)的点积
r=dotmultiply(p1,p2,op),得到矢量(p1-op)和(p2-op)的点积
r < 0: 两矢量夹角为锐角；
r = 0：两矢量夹角为直角；
r > 0: 两矢量夹角为钝角
*/
/* dot product : (p1-op)(p2-op)
 * intersection angle:
 * r < 0 : acute angle
 * r = 0 : right angle
 * r > 0 : obtuse angle
 */
qreal dotmultiply(const QPointF &p1, const QPointF &p2, const QPointF &p0)
{
	return ((p1.x() - p0.x()) * (p2.x() - p0.x()) + (p1.y() - p0.y()) * (p2.y() - p0.y()));
}

// 返回两点之间距离
// distance between p1 and p2
qreal p2pdist(const QPointF &p1, const QPointF &p2)
{
	return(sqrt((p1.x() - p2.x()) * (p1.x() - p2.x()) + (p1.y() - p2.y()) * (p1.y() - p2.y())));
}

/* 点和线段关系
qreal < 0 则c超过线l的p1端
qreal > 1 则c超过线l的p2端
0 < qreal < 1 则垂足在线段内
*/
qreal p2lRelation(const QPointF &c, const QLineF &l)
{
//	QLineF tl;
//	tl.setP1(l.p1());
//	tl.setP2(c);
	return dotmultiply(c, l.p2(), l.p1()) / (p2pdist(l.p1(), l.p2()) * p2pdist(l.p1(), l.p2()));
}


// 求点p到线段l所在直线的垂足
// foot point : from point p to line l
QPointF perpendicular(const QPointF &p, const QLineF &l)
{
	qreal r=p2lRelation(p, l);
	QPointF tp;
	qreal x = l.p1().x() + r * (l.p2().x() - l.p1().x());
	tp.setX(x);
	qreal y = l.p1().y() + r * (l.p2().y() - l.p1().y());
	tp.setY(y);
	return tp;
}

// 求点p到线段l的最近的点np
// np是线段l上到点p最近的点，不一定是垂足

/* The nearest point from line l to point p
 */
QPointF p2lMinPos(const QPointF &p, const QLineF &l)
{
	QPointF np;
	qreal r = p2lRelation(p, l);
	if(r < 0)
	{
		np = l.p1();
	}
	else if(r>1)
	{
		np = l.p2();
	}
	else
		np = perpendicular(p, l);
	return np;
}


/*
(sp-op)*(ep-op)的叉积
r=multicross(sp,ep,op),得到(sp-op)*(ep-op)的叉积
r>0:sp在矢量op ep的顺时针方向；
r=0：op sp ep三点共线；
r<0: sp在矢量op ep的逆时针方向
*/
/* cross product : (sp-op)*(ep-op)
 * r > 0 : sp in a clockwise direction of vector op and ep
 * r = 0 : collineation(op, sp, ep)
 * r < 0 : sp in a counter-clockwise direction of vector op and ep
 */
qreal multicross(const QPointF &sp, const QPointF &ep, const QPointF &op)
{
	return((sp.x() - op.x()) * (ep.y() - op.y()) - (ep.x() - op.x()) * (sp.y() - op.y()));
}

// 点到线垂线
// vertical line
qreal p2ldist(const QPointF &p, const QLineF &l)
{
	return fabs(multicross(p, l.p2(), l.p1())) / p2pdist(l.p1(), l.p2());
}


// 点到三角形最近的点, abc 为三角形三个顶点
// the nearest point from p to triangle(pa, pb, pc)
QPointF p2triangleMinPos(const QPointF &p, const QPointF &pa, const QPointF &pb, const QPointF &pc)
{
	qreal p2pa = p2pdist(p, pa);
	qreal p2pb = p2pdist(p, pb);
	qreal p2pc = p2pdist(p, pc);
	QLineF line;
	QPointF p1, p2;
	if (p2pa < p2pb)
	{
		p1 = pa;
		p2 = p2pb < p2pc ? pb : pc;
	}
	else
	{
		p1 = pb;
		p2 = p2pa < p2pc ? pa : pc;
	}
	line.setP1(p1);
	line.setP2(p2);
	
	QPointF mPos = p2lMinPos(p, line);
	return mPos;
}


// 判断点是否在三角形内, 定理： 同向(叉积运算)
// whether the point within the triangle
bool calTriangleContainsPt(const QPointF &p, const QPointF &a, const QPointF &b, const QPointF &c)
{
	qreal x = p.x();
	qreal y = p.y();
	// 求出点角 向量
	qreal XA1 = x - a.x();
	qreal XA2 = y - a.y();
	qreal XB1 = x - b.x();
	qreal XB2 = y - b.y();
	qreal XC1 = x - c.x();
	qreal XC2 = y - c.y();
	
	// cross product
	qreal XA2XB = XA1 * XB2 - XB1 * XA2;
	qreal XB2XC = XB1 * XC2 - XC1 * XB2;
	qreal XC2XA = XC1 * XA2 - XA1 * XC2;
	
	bool bInTriangle = false;
	if (XA2XB > 0 && XB2XC > 0 && XC2XA > 0)
		bInTriangle = true;
	
	if (XA2XB < 0 && XB2XC < 0 && XC2XA < 0)
		bInTriangle = true;
	return bInTriangle;
}


// 弧度
// radian
qreal radianAt(const QPointF &pos, const QRect &rect)
{
	qreal mousexdist = pos.x() - (qreal) rect.center().x();
	qreal mouseydist = pos.y() - (qreal) rect.center().y();
	qreal mouserad = sqrt(mousexdist * mousexdist + mouseydist * mouseydist);
	if (qFuzzyCompare(mouserad, 0.0))
		return 0.0;
	
	qreal angle = acos(mousexdist / mouserad);
	if (mouseydist >= 0)
		angle = HSVTWOPI - angle;
	
	return angle;
}


KColorCircleHsv::KColorCircleHsv(QWidget *parent)
	: QWidget(parent), m_imgBG(sizeHint(), QImage::Format_RGB32), m_selMode(None)
{
	setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
	setFocusPolicy(Qt::StrongFocus);
	setMinimumSize(100, 100);
	m_bNeedUpdateBackground = true;
	m_nCurrentHue = 0;
	calRadian(m_nCurrentHue);
	QColor tmp;
	tmp.setHsv(0, 0, 0);
	setColor(tmp);
}


KColorCircleHsv::~KColorCircleHsv()
{
}


void KColorCircleHsv::calRadian(int hue)
{
	m_radA = (((360 - hue) * HSVTWOPI) / 360.0);
	m_radA += HSVPI / 2.0;
	if (m_radA > HSVTWOPI)
		m_radA -= HSVTWOPI;
	
	m_radB = m_radA + HSVTWOPI/3;
	m_radC = m_radB + HSVTWOPI/3;
	
	if (m_radB > HSVTWOPI)
		m_radB -= HSVTWOPI;
	if (m_radC > HSVTWOPI)
		m_radC -= HSVTWOPI;
	
	calVertexPoint();
}

// 计算顶点 (calculate vertext)
// a : 颜色顶点
// b : s = 0 v = 0
// c : s = 0 v = 255
void KColorCircleHsv::calVertexPoint()
{
	qreal cx = (qreal) contentsRect().center().x();
	qreal cy = (qreal) contentsRect().center().y();
	int innerRadius = m_nOuterRadius - m_dOuterInnerWidth;
	pa = QPointF(cx + (cos(m_radA) * innerRadius), cy - (sin(m_radA) * innerRadius));
	pb = QPointF(cx + (cos(m_radB) * innerRadius), cy - (sin(m_radB) * innerRadius));
	pc = QPointF(cx + (cos(m_radC) * innerRadius), cy - (sin(m_radC) * innerRadius));
	pd = QPointF(pa.x() + cos(m_radA) * m_dOuterInnerWidth, 
				 pa.y() - (sin(m_radA) * m_dOuterInnerWidth));
	
}


//  背景图，仅画圆环
// drawing background image
void KColorCircleHsv::createBackground()
{
	qreal innerRadius = m_nOuterRadius - m_dOuterInnerWidth;
	m_imgBG = QImage(contentsRect().size(), QImage::Format_RGB32);
	QPainter p(&m_imgBG);
	p.setRenderHint(QPainter::Antialiasing);
	p.fillRect(m_imgBG.rect(), palette().background());
	
	QConicalGradient gradient(m_imgBG.rect().center(), 90);
	QColor color;
	// 色环：360～0：red-green-blue-red逆时针，90度为起点
	for (qreal i = 0; i <= 1.0; i += 0.1) 
	{
		color.setHsv(int(360.0 - (i * 360.0)), 255, 255);	//一个纯色相pure hue
		gradient.setColorAt(i, color);
	}

	// 画环形
	QRectF innerRadiusRect(m_imgBG.rect().center().x() - innerRadius, m_imgBG.rect().center().y() - innerRadius,
							innerRadius * 2 + 1, innerRadius * 2 + 1);
	QRectF outerRadiusRect(m_imgBG.rect().center().x() - m_nOuterRadius, m_imgBG.rect().center().y() - m_nOuterRadius,
							m_nOuterRadius * 2 + 1, m_nOuterRadius * 2 + 1);
	QPainterPath path;
	path.addEllipse(innerRadiusRect);
	path.addEllipse(outerRadiusRect);

	p.save();
	p.setClipPath(path);
	p.fillRect(m_imgBG.rect(), gradient);
	p.restore();
	
}


// 鼠标坐标改变
// mouse point changed
bool KColorCircleHsv::pointChanged(QPointF point)
{
	bool newColor = false;
	if (m_selMode == SelCircle)
	{
		// 更新顶点
		// m_radA is hue: 选中点平面坐标系的弧度
		m_radA = radianAt(point, contentsRect());
		// b is black: m_radA/120, , 对逆时针旋转120度，
		m_radB = m_radA + HSVTWOPI / 3.0;
		m_radC = m_radB + HSVTWOPI / 3.0;
		if (m_radB > HSVTWOPI) m_radB -= HSVTWOPI;
		if (m_radC > HSVTWOPI) m_radC -= HSVTWOPI;
		qreal am = m_radA - HSVPI/2;
		if (am < 0) am += HSVTWOPI;
		int te = (int) (((am) * 360.0) / (HSVTWOPI));
		m_nCurrentHue = 360 - te;
		
		int h,s,v;
		m_CurrentColor.getHsv(&h, &s, &v);
		if (m_nCurrentHue != h) 
		{
			newColor = true;
			m_CurrentColor.setHsv(m_nCurrentHue, s, v);
		}
		calVertexPoint();
		m_dSelectorPos = pointFromColor(m_CurrentColor);
	}
	else if(m_selMode == SelTriangle)
	{
		// 是否在三角形内
		QPointF fpos = point;
		if (!calTriangleContainsPt(point, pa, pb, pc))
			fpos = p2triangleMinPos(point, pa, pb, pc);
		
		m_dSelectorPos = fpos;
		QColor col = colorFromPoint(m_dSelectorPos);
		if (col != m_CurrentColor) 
		{
			int h, s, v;
			col.getHsv(&h, &s, &v);
			m_CurrentColor.setHsv(m_nCurrentHue, s, v);
			newColor = true;
		}
	}
	
	return newColor;
}

void KColorCircleHsv::mouseMoveEvent(QMouseEvent *e)
{
	if ((e->buttons() & Qt::LeftButton) == 0)
		return;
	
	QPointF fpos = e->posF();
	bool newColor = this->pointChanged(fpos);
	if (newColor)
		emit colorChanged(m_CurrentColor);
	
	update();
}


void KColorCircleHsv::mousePressEvent(QMouseEvent *e)
{
	if (e->button() != Qt::LeftButton)
		return;
	QPointF fPos = e->posF();
	qreal rad = p2pdist(fPos, contentsRect().center());
	if (rad > (m_nOuterRadius - m_dOuterInnerWidth))
	{
		m_selMode = SelCircle;
	}
	else
	{
		// 是否在三角形内
		if (calTriangleContainsPt(fPos, pa, pb, pc))
			m_selMode = SelTriangle;
		else
			m_selMode = None;
	}
	
	bool newColor = this->pointChanged(fPos);
	if (newColor)
		emit colorChanged(m_CurrentColor);
	
	update();
}

void KColorCircleHsv::mouseReleaseEvent(QMouseEvent *e)
{
	if (e->buttons() & Qt::LeftButton)
		m_selMode = None;
}

void KColorCircleHsv::keyPressEvent(QKeyEvent *e)
{
	switch (e->key()) 
	{
		case Qt::Key_Left:
		{
			--m_nCurrentHue;
			if (m_nCurrentHue < 0) m_nCurrentHue += 360;
			int h,s,v;
			m_CurrentColor.getHsv(&h, &s, &v);
			QColor tmp;
			tmp.setHsv(m_nCurrentHue, s, v);
			setColor(tmp);
			emit colorChanged(m_CurrentColor);
		}
		break;
		case Qt::Key_Right:
		{
			++m_nCurrentHue;
			if (m_nCurrentHue > 359) m_nCurrentHue -= 360;
			int h,s,v;
			m_CurrentColor.getHsv(&h, &s, &v);
			QColor tmp;
			tmp.setHsv(m_nCurrentHue, s, v);
			setColor(tmp);
			emit colorChanged(m_CurrentColor);
		}
		break;
		case Qt::Key_Up:
		{
			int h,s,v;
			m_CurrentColor.getHsv(&h, &s, &v);
			QColor tmp;
			if (v > 5) 
				v -= 5;
			else 
				v = 0;
			tmp.setHsv(m_nCurrentHue, s, v);
			setColor(tmp);
			emit colorChanged(m_CurrentColor);
		}
		break;
		case Qt::Key_Down:
		{
			int h,s,v;
			m_CurrentColor.getHsv(&h, &s, &v);
			QColor tmp;
			if (v < 250) 
				v += 5;
			else 
				v = 255;
			tmp.setHsv(m_nCurrentHue, s, v);
			setColor(tmp);
			emit colorChanged(m_CurrentColor);
		}
		break;
	};
}


void KColorCircleHsv::resizeEvent(QResizeEvent *)
{
	m_nOuterRadius = (contentsRect().width() - 1) / 2;
	if ((contentsRect().height() - 1) / 2 < m_nOuterRadius)
		m_nOuterRadius = (contentsRect().height() - 1) / 2;
	
	m_nPenWidth = (int) floor(m_nOuterRadius / 50.0);
	m_nSVEllipseSize = (int) floor(m_nOuterRadius / 12.5);
	m_dOuterInnerWidth = m_nOuterRadius / 5.0;
	
	calVertexPoint();
	
	m_dSelectorPos = pointFromColor(m_CurrentColor);
	m_bNeedUpdateBackground = true;
	paintImage();
	update();
}


void KColorCircleHsv::paintEvent(QPaintEvent *e)
{
	QPainter p(this);
	p.setRenderHint(QPainter::Antialiasing);
	if (e->rect().intersects(contentsRect()))
		p.setClipRegion(e->region().intersect(contentsRect()));
	
	if (m_CurrentColor != m_OldColor)
	{
		paintImage();
		m_OldColor = m_CurrentColor;
	}
	p.drawImage(contentsRect().topLeft(), m_buf);
}


// 缓冲图
// cache image
void KColorCircleHsv::paintImage()
{
	if (m_bNeedUpdateBackground) 
	{
		createBackground();
		m_bNeedUpdateBackground = false;
	}
	
	m_buf = m_imgBG.copy();
	
	// ########  三角形
	// pure hue
	QColor hueColor;
	hueColor.setHsv(m_nCurrentHue, 255, 255);
	drawTriangle(&m_buf, pa, pb, pc, hueColor);
	
	//QPixmap pix = QPixmap::fromImage(m_buf);
	QPainter painter(&m_buf);
	painter.setRenderHint(QPainter::Antialiasing);
	
	// ##### 画hue定位线
	int ri, gi, bi;
	hueColor.getRgb(&ri, &gi, &bi);
	// 混合RGB通道 ：red*30% + green*59% + blue*11%=255
	// 参考http://www.gimp.org/tutorials/Color2BW/
	if ((ri * 30) + (gi * 59) + (bi * 11) > 12800)
		painter.setPen(QPen(Qt::black, m_nPenWidth));
	else
		painter.setPen(QPen(Qt::white, m_nPenWidth));
	// 反色效果
	//painter.setPen(QPen(QColor(255-ri, 255- gi, 255-bi), m_nPenWidth, Qt::SolidLine, Qt::RoundCap));
	
//	painter.drawEllipse((int) (pd.x() - m_nSVEllipseSize / 2.0),
//			(int) (pd.y() - m_nSVEllipseSize / 2.0),
//			m_nSVEllipseSize, m_nSVEllipseSize);
	painter.drawLine(pa, pd);
	
	// ##### 画s v定位圈
	painter.setPen(QPen(QColor(255-ri, 255- gi, 255-bi), m_nPenWidth, Qt::SolidLine, Qt::RoundCap));
	
	painter.drawEllipse(QRectF(m_dSelectorPos.x() - m_nSVEllipseSize / 2.0,
							   m_dSelectorPos.y() - m_nSVEllipseSize / 2.0,
							   m_nSVEllipseSize + 0.5, m_nSVEllipseSize + 0.5));
}


// 画三角形SV
// TODO    --->  消除锯齿(jagged)
void KColorCircleHsv::drawTriangle(QImage *buf, const QPointF &pa,
								const QPointF &pb, const QPointF &pc,
								const QColor &color)
{
	Vertex aa(color, pa);
	Vertex bb(Qt::black, pb);
	Vertex cc(Qt::white, pc);
	
	// 冒泡sort
	// Y : aa < bb < cc.
	if (aa.point.y() > bb.point.y())
		std::swap(aa, bb);
	if (aa.point.y() > cc.point.y())
		std::swap(aa, cc);
	if (bb.point.y() > cc.point.y())
		std::swap(bb, cc);
	
	qreal aabbydist = bb.point.y() - aa.point.y();
	qreal aaccydist = cc.point.y() - aa.point.y();
	qreal bbccydist = cc.point.y() - bb.point.y();
	qreal aabbxdist = bb.point.x() - aa.point.x();
	qreal aaccxdist = cc.point.x() - aa.point.x();
	qreal bbccxdist = cc.point.x() - bb.point.x();

	// 左三角： bb.x < aa.x : 
	bool lefty = aabbxdist < 0;
	
	QVarLengthArray<DoubleColor, 640> leftColors;
	QVarLengthArray<DoubleColor, 640> rightColors;
	QVarLengthArray<qreal, 640> leftX; 
	QVarLengthArray<qreal, 640> rightX; 
	int nSize = int(floor(cc.point.y() + 1));
	leftColors.resize(nSize);
	rightColors.resize(nSize);
	leftX.resize(nSize);
	rightX.resize(nSize);
	
	DoubleColor source;
	DoubleColor dest;
	qreal r, g, b;
	qreal rdelta, gdelta, bdelta;
	qreal x;
	qreal xdelta;
	int y1, y2;

	// 
	x = aa.point.x();
	source = aa.color;
	dest = cc.color;
	r = source.r;
	g = source.g;
	b = source.b;
	y1 = (int) floor(aa.point.y());
	y2 = (int) floor(cc.point.y());
	
	// delta
	xdelta = aaccxdist / aaccydist;
	rdelta = (dest.r - r) / aaccydist;
	gdelta = (dest.g - g) / aaccydist;
	bdelta = (dest.b - b) / aaccydist;
	
	// 线性渐变
	int y;
	for (y = y1; y < y2; ++y) 
	{
		if (lefty) 
		{
			rightColors[y] = DoubleColor(r, g, b);
			rightX[y] = x;
		}
		else
		{
			leftColors[y] = DoubleColor(r, g, b);
			leftX[y] = x;
		}
		
		r += rdelta;
		g += gdelta;
		b += bdelta;
		x += xdelta;
	}
	
	x = aa.point.x();
	source = aa.color;
	dest = bb.color;
	r = source.r;
	g = source.g;
	b = source.b;
	y1 = (int) floor(aa.point.y());
	y2 = (int) floor(bb.point.y());
	
	xdelta = aabbxdist / aabbydist;
	rdelta = (dest.r - r) / aabbydist;
	gdelta = (dest.g - g) / aabbydist;
	bdelta = (dest.b - b) / aabbydist;
	
	for (y = y1; y < y2; ++y)
	{
		if (lefty)
		{
			leftColors[y] = DoubleColor(r, g, b);
			leftX[y] = x;
		}
		else
		{
			rightColors[y] = DoubleColor(r, g, b);
			rightX[y] = x;
		}
		
		r += rdelta;
		g += gdelta;
		b += bdelta;
		x += xdelta;
	}
	
	x = bb.point.x();
	source = bb.color;
	dest = cc.color;
	r = source.r;
	g = source.g;
	b = source.b;
	y1 = (int) floor(bb.point.y());
	y2 = (int) floor(cc.point.y());
	
	xdelta = bbccxdist / bbccydist;
	rdelta = (dest.r - r) / bbccydist;
	gdelta = (dest.g - g) / bbccydist;
	bdelta = (dest.b - b) / bbccydist;
	
	for (y = y1; y < y2; ++y)
	{
		if (lefty)
		{
			leftColors[y] = DoubleColor(r, g, b);
			leftX[y] = x;
		}
		else
		{
			rightColors[y] = DoubleColor(r, g, b);
			rightX[y] = x;
		}
		
		r += rdelta;
		g += gdelta;
		b += bdelta;
		x += xdelta;
	}
	
	// 从上到下，从左到右扫描buf，在(pa,pb,pc范围内)
	// 从左到右：从leftColor到rightColor做线性变换
	//	从 上到下
	const int ccyfloor = int(floor(cc.point.y()));
	for (int y = int(floor(aa.point.y())); y < ccyfloor; ++y)
	{
		qreal lx = leftX[y];
		qreal rx = rightX[y];
		
		int lxi = (int) floor(lx);
		int rxi = (int) floor(rx);
		DoubleColor rc = rightColors[y];
		DoubleColor lc = leftColors[y];
		
		double xdist = rx - lx;
		if (!qFuzzyCompare(xdist, 0.0))
		{
			qreal r = lc.r;
			qreal g = lc.g;
			qreal b = lc.b;
			qreal rdelta = (rc.r - r) / xdist;
			qreal gdelta = (rc.g - g) / xdist;
			qreal bdelta = (rc.b - b) / xdist;
			
			QRgb *scanline = reinterpret_cast<QRgb *>(buf->scanLine(y));
			scanline += lxi;
			
			// 从左到右
			for (int i = lxi; i < rxi; ++i)
			{
				*scanline++ = qRgb((int) r, (int) g, (int) b);
				r += rdelta;
				g += gdelta;
				b += bdelta;
			}
		}
	}
}


void KColorCircleHsv::setColor(const QColor &col)
{
	if (col.toHsl() == m_CurrentColor.toHsl())
		return;
	
	int oldhue = m_CurrentColor.hslHue();
	m_CurrentColor = col;
	
	if (oldhue != col.hslHue())
	{
		int hsvHue = col.hsvHue();
		int hslHue = col.hslHue();
		if (hsvHue != -1)
		{
			m_nCurrentHue = hsvHue;
			calRadian(m_nCurrentHue);
		}
		else if (hsvHue == -1 && hslHue != -1)
		{
			m_nCurrentHue = hslHue;
			calRadian(m_nCurrentHue);
		}
		
	}
	m_dSelectorPos = pointFromColor(m_CurrentColor);
	
	update();
}

void KColorCircleHsv::setColor(qreal h, qreal s, qreal l)
{
	QColor col = KColorConverter::fromHsl(h, s, l);
	setColor(col);
}



QColor KColorCircleHsv::color() const
{
	return m_CurrentColor;
}


QPointF KColorCircleHsv::pointFromColor(const QColor &col) const
{
	
	if (col == Qt::black)
		return pb;
	else if (col == Qt::white)
		return pc;
	
	// 
	qreal abX = pb.x() - pa.x();
	qreal abY = pb.y() - pa.y();
	qreal bcX = pc.x() - pb.x();
	qreal bcY = pc.y() - pb.y();
	qreal acX = pc.x() - pa.x();
	qreal acY = pc.y() - pa.y();
	
	int hue, sat, val;
	//qreal hue,sat,val;
	col.getHsv(&hue, &sat, &val);
	//KColorConverter::getHsl(col, &hue, &sat, &val);
	
	// 饱和度：a到c（100%-0）渐变的，可以理解为垂直于ac轴
	// 亮度：a and c的亮度是一样的，所以亮度就是由b到a和c进行渐变的，可以理解于平行于ac
	// 求出亮度和饱和度相交就是色彩的位置了。
	
	// a到b，b到c：亮度渐变
	// color to black ： color ~ 0
	// a 到 b渐变过程中
	qreal abxV = pa.x() + (abX * (qreal) (255 - val)) / 255.0;
	qreal abyV = pa.y() + (abY * (qreal) (255 - val)) / 255.0;
	// black to white : 0 ～ color
	// b to c 渐变
	qreal bcxV = pb.x() + (bcX * (qreal) val) / 255.0;
	qreal bcyV = pb.y() + (bcY * (qreal) val) / 255.0;
	
	// a到c：饱和度渐变
	// white to color : s由color to 0
	// 
	qreal acxS = pa.x() + (acX * (qreal) (255 - sat)) / 255.0;
	qreal acyS = pa.y() + (acY * (qreal) (255 - sat)) / 255.0;
	qreal p4 = pb.x();
	qreal q4 = pb.y();
	
	qreal x = 0;
	qreal y = 0;
	if (!qFuzzyCompare(abxV-bcxV, 0.0))
	{
		qreal a = (bcyV - abyV) / (bcxV - abxV);	// 亮度线的斜率
		qreal c = (q4 - acyS) / (p4 - acxS);		// 饱和线斜率
		qreal b = abyV - a * abxV;
		qreal d = acyS - c * acxS;
		// 求交点
		x = (d - b) / (a - c);
		y = a * x + b;
	}
	else {
		x = abxV;
		y = acyS + (x - acxS) * (q4 - acyS) / (p4 - acxS);
	}
	
	return QPointF(x, y);
}


// 
QColor KColorCircleHsv::colorFromPoint(const QPointF &p)  const
{
	// valuef(亮度比)
	qreal a2b = p2pdist(pa, pb);
	qreal b2ac = a2b * sin(60.0 * HSVPI / 180.0);
	
	// p to ac
	qreal p2ac = p2ldist(p, QLineF(pa, pc));
	qreal valuef = (b2ac - p2ac) / b2ac;
	
	qreal a2c = p2pdist(pa, pc);
	qreal p2c = p2pdist(p, pc);
	qreal m2c = pythagorean(p2c, p2ac, true);
	qreal satf = m2c / a2c; 
	
	valuef = qBound(0.0, valuef, 1.0);
	satf = qBound(0.0, satf, 1.0);
	QColor c = QColor::fromHsvF(m_nCurrentHue / 360.0, satf, valuef);
	return c;
}
