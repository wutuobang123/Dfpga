//=============================================================================
// ticao_qt - 体态筛查原生看板（Qt5 Widgets，全屏无边框）
//   数据源：/home/kickpi/projects/ticao/live.json（流水线每帧更新）
//   显示：总分 / 星级 / 当前动作与进度 / 纠错提示 / 四项得分
//=============================================================================
#include <QApplication>
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFont>
#include <QDateTime>
#include <QElapsedTimer>
#include <QScreen>

static const char *LIVE_PATH = "/home/kickpi/projects/ticao/live.json";

class Board : public QWidget {
public:
    Board() {
        // 绕过窗口管理器：override-redirect 窗口总是盖在所有管理窗口（含桌面面板）之上
        setWindowFlags(Qt::Window | Qt::FramelessWindowHint
                       | Qt::WindowStaysOnTopHint | Qt::X11BypassWindowManagerHint);
        setCursor(Qt::BlankCursor);
        setStyleSheet("background:#0d141c;");
        setWindowTitle("体态筛查");

        m_timer = new QTimer(this);
        connect(m_timer, &QTimer::timeout, this, &Board::reload);
        m_timer->start(200);
        m_clock.start();
        m_raise = new QTimer(this);
        connect(m_raise, &QTimer::timeout, this, [this]{ raise(); });
        m_raise->start(1000);
    }

protected:
    void reload() {
        QFile f(QString::fromUtf8(LIVE_PATH));
        if (!f.open(QIODevice::ReadOnly)) return;
        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
        f.close();
        if (err.error != QJsonParseError::NoError || !doc.isObject()) return;
        m_obj = doc.object();
        m_ok = true;
        update();
    }

    void paintEvent(QPaintEvent *) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.setRenderHint(QPainter::TextAntialiasing, true);

        const int W = width(), H = height();
        p.fillRect(rect(), QColor(13, 20, 28));

        QFont f = font();
        f.setFamily("WenQuanYi Zen Hei");
        p.setFont(f);

        // ---------- 顶栏 ----------
        p.setPen(QColor(120, 160, 200));
        QFont ft = f; ft.setPixelSize(H / 26); ft.setBold(true);
        p.setFont(ft);
        p.drawText(QRect(W / 40, H / 40, W / 2, H / 20), Qt::AlignLeft | Qt::AlignVCenter,
                   QString::fromUtf8("少儿体态筛查 · 上半身"));
        QFont fs = f; fs.setPixelSize(H / 40);
        p.setFont(fs);
        p.setPen(QColor(90, 120, 150));
        p.drawText(QRect(W / 2, H / 40, W / 2 - W / 40, H / 20), Qt::AlignRight | Qt::AlignVCenter,
                   QString::fromUtf8("v1.0   开机自启"));

        if (!m_ok) {
            QFont fw = f; fw.setPixelSize(H / 18);
            p.setFont(fw);
            p.setPen(QColor(150, 170, 190));
            p.drawText(rect(), Qt::AlignCenter, QString::fromUtf8("等待数据…"));
            return;
        }

        const QJsonObject &o = m_obj;
        double score  = o.value("score").toDouble();
        int    stars  = o.value("stars").toInt();
        int    step   = o.value("step").toInt();
        int    total  = o.value("total").toInt();
        double prog   = o.value("progress").toDouble();
        if (total > 0 && step > total) step = total;
        if (total > 0 && step < 0)     step = 0;
        bool   det    = o.value("det").toBool();
        QString action = o.value("action_name").toString();
        QString hint   = o.value("hint").toString();
        QJsonObject sum = o.value("summary").toObject();
        bool finished = !sum.value("items").toArray().isEmpty();
        if (finished) prog = 100.0;

        // ---------- 左：总分 ----------
        const int LX = W / 24, LY = H / 8;
        const int LW = W * 2 / 5;

        QFont fnum = f; fnum.setPixelSize(H / 4); fnum.setBold(true);
        p.setFont(fnum);
        p.setPen(QColor(90, 220, 160));
        p.drawText(QRect(LX, LY, LW, H / 3), Qt::AlignLeft | Qt::AlignVCenter,
                   QString::number((int)qRound(score)));

        QFont funit = f; funit.setPixelSize(H / 16);
        p.setPen(QColor(120, 150, 175));
        QFontMetrics fmBig(fnum);
        int numW = fmBig.horizontalAdvance(QString::number((int)qRound(score)));
        p.setFont(funit);
        QFontMetrics fmUnit(p.font());
        int unitY = LY + (H / 4 - fmUnit.height()) / 2 + H / 12;
        p.drawText(LX + numW + W / 90, unitY,
                   QString::fromUtf8("分"));

        // 星级
        QFont fstar = f; fstar.setPixelSize(H / 12);
        p.setFont(fstar);
        for (int i = 0; i < 3; ++i) {
            p.setPen(i < stars ? QColor(255, 200, 60) : QColor(55, 70, 85));
            p.drawText(QRect(LX + i * H / 10, LY + H / 3 - H / 30, H / 10, H / 12),
                       Qt::AlignCenter, QString::fromUtf8("★"));
        }

        // ---------- 右：当前动作 + 进度 ----------
        const int RX = W / 2 + W / 24;
        const int RW = W / 2 - W / 12;

        QFont fa = f; fa.setPixelSize(H / 16); fa.setBold(true);
        p.setFont(fa);
        p.setPen(QColor(230, 240, 250));
        QString title = action.isEmpty() ? QString::fromUtf8("准备中") : action;
        if (finished)      title = QString::fromUtf8("筛查完成   %1/%1").arg(total);
        else if (total > 0) title += QString("   %1/%2").arg(step).arg(total);
        p.drawText(QRect(RX, LY, RW, H / 10), Qt::AlignLeft | Qt::AlignVCenter, title);

        // 进度条
        const int barY = LY + H / 8, barH = H / 40;
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(30, 42, 55));
        p.drawRoundedRect(QRect(RX, barY, RW, barH), barH / 2, barH / 2);
        p.setBrush(QColor(70, 200, 150));
        int fillW = (int)(RW * qBound(0.0, prog / 100.0, 1.0));
        p.drawRoundedRect(QRect(RX, barY, fillW, barH), barH / 2, barH / 2);

        // 提示语
        QFont fh = f; fh.setPixelSize(H / 20);
        p.setFont(fh);
        p.setPen(det ? QColor(255, 230, 150) : QColor(230, 120, 120));
        QString h = hint.isEmpty()
                    ? (det ? QString::fromUtf8("请保持姿势") : QString::fromUtf8("请站到画面中"))
                    : hint;
        p.drawText(QRect(RX, barY + barH + H / 40, RW, H / 8),
                   Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap, h);

        // ---------- 底部：四项得分 ----------
        const int BY = H * 2 / 3;
        QJsonArray items = sum.value("items").toArray();
        if (items.isEmpty()) {
            QFont fb = f; fb.setPixelSize(H / 26);
            p.setFont(fb);
            p.setPen(QColor(90, 115, 140));
            p.drawText(QRect(W / 24, BY, W - W / 12, H / 6), Qt::AlignLeft | Qt::AlignTop,
                       QString::fromUtf8("完成后这里显示各项评估结果"));
        } else {
            int n = items.size();
            int gap = W / 60;
            int cw = (W - W / 12 - gap * (n - 1)) / n;
            for (int i = 0; i < n; ++i) {
                QJsonObject it = items.at(i).toObject();
                int cx = W / 24 + i * (cw + gap);
                QRect card(cx, BY, cw, H / 4);
                p.setPen(Qt::NoPen);
                p.setBrush(QColor(22, 32, 44));
                p.drawRoundedRect(card, H / 60, H / 60);

                double s = it.value("score").toDouble();
                p.setPen(QColor(70, 200, 150));
                QFont f1 = f; f1.setPixelSize(H / 13); f1.setBold(true);
                p.setFont(f1);
                p.drawText(QRect(cx, BY + H / 40, cw, H / 10), Qt::AlignCenter,
                           QString::number((int)qRound(s)));

                p.setPen(QColor(190, 205, 220));
                QFont f2 = f; f2.setPixelSize(H / 30);
                p.setFont(f2);
                QString nm = it.value("pose").toString();
                if (nm.isEmpty()) nm = it.value("name").toString();
                p.drawText(QRect(cx + H / 80, BY + H / 7, cw - H / 40, H / 8),
                           Qt::AlignHCenter | Qt::AlignTop | Qt::TextWordWrap, nm);
            }
        }

        // 底部状态行
        QFont fst = f; fst.setPixelSize(H / 42);
        p.setFont(fst);
        p.setPen(QColor(70, 92, 112));
        p.drawText(QRect(W / 24, H - H / 20, W - W / 12, H / 24),
                   Qt::AlignLeft | Qt::AlignVCenter,
                   QString::fromUtf8("推理 %1fps   ·   %2")
                       .arg(o.value("infer_fps").toDouble(), 0, 'f', 1)
                       .arg(QDateTime::currentDateTime().toString("hh:mm:ss")));
    }

private:
    QTimer *m_timer = nullptr;
    QTimer *m_raise = nullptr;
    QJsonObject m_obj;
    bool m_ok = false;
    QElapsedTimer m_clock;
};

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    QApplication::setOverrideCursor(Qt::BlankCursor);
    Board b;
    b.setGeometry(QGuiApplication::primaryScreen()->geometry());
    b.show();            // 配合 bypass hint，手动铺满整个屏幕
    b.raise();
    return app.exec();
}