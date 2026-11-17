//! The run toast: what the last background run did, under the top bar's right end, while the screen on show is not Simulate
//! (which shows the results itself). It is not modal: no scrim, only the panel takes clicks.

use iced::widget::text::Wrapping;
use iced::widget::{column, container, mouse_area, row, text, Space};
use iced::{Center, Element, Fill, Padding, Widget};

use super::Message as ChromeMessage;
use crate::app::{self, Ctx, Mode, RunLine, RunReport};
use crate::theme;
use crate::tokens::{SPACE_1, SPACE_2, SPACE_3, TEXT_BODY, TOPBAR_H};
use crate::units::{self, Quantity};
use crate::widgets::button::{button, icon_button, Weight};
use crate::widgets::glyph::{icon, Mark, Tone};

/// Width of the panel.
const TOAST_W: f32 = 320.0;
/// Flights listed before `+N more`.
const MAX_ROWS: usize = 4;
/// The mono size of a flight's numbers.
const STATS_SIZE: f32 = 11.5;

/// The toast layer, when there is a report to show and the screen is not Simulate.
pub fn run_toast(ctx: &Ctx) -> Option<Element<'_, app::Message>> {
    let report = ctx.run.as_ref()?;
    if ctx.mode == Mode::Simulate {
        return None;
    }
    let panel = container(mouse_area(content(report, ctx)).on_press(app::Message::Chrome(ChromeMessage::Nothing))).width(TOAST_W).style(theme::overlay);
    Some(
        container(panel)
            .width(Fill)
            .height(Fill)
            .padding(Padding { top: TOPBAR_H + SPACE_2, right: SPACE_3, ..Padding::ZERO })
            .align_right(Fill)
            .align_top(Fill)
            .boxed(),
    )
}

/// The title: `Flying 3 flights…`, `3 flights flown`, `2 of 3 flights flown`.
fn title(report: &RunReport) -> String {
    let n = report.lines.len();
    let flights = if n == 1 { "flight" } else { "flights" };
    if report.pending > 0 {
        format!("Flying {n} {flights}…")
    } else if report.failed() > 0 {
        format!("{} of {n} flights flown", n - report.failed())
    } else {
        format!("{n} {flights} flown")
    }
}

fn hairline<'a>() -> Element<'a, app::Message> {
    container(Space::new()).width(Fill).height(1).style(theme::hairline).boxed()
}

fn content<'a>(report: &'a RunReport, ctx: &'a Ctx) -> Element<'a, app::Message> {
    let failed = report.failed();
    let (mark, tone) = if report.pending > 0 {
        (Mark::Play, Tone::Muted)
    } else if failed == 0 {
        (Mark::Check, Tone::Ok)
    } else if failed == report.lines.len() {
        (Mark::Error, Tone::Err)
    } else {
        (Mark::Warning, Tone::Warn)
    };
    let close: Element<'a, app::Message> = icon_button(Mark::Close, "Dismiss").on_press(app::Message::DismissRun).into();
    let head = row![icon(mark, tone, 14.0), text(title(report)).size(TEXT_BODY).style(theme::ink).width(Fill), close].spacing(SPACE_2).align_y(Center);

    let mut body = column![container(head).padding([SPACE_1, SPACE_3])];
    for line in report.lines.iter().take(MAX_ROWS) {
        body = body.push(hairline()).push(flight_row(line, ctx));
    }
    if report.lines.len() > MAX_ROWS {
        let more = text(format!("+{} more", report.lines.len() - MAX_ROWS)).size(STATS_SIZE).style(theme::faint);
        body = body.push(hairline()).push(container(more).padding([SPACE_1, SPACE_3]).boxed());
    }
    let open: Element<'a, app::Message> = button("Open Simulate", Weight::Secondary).on_press(app::Message::Mode(Mode::Simulate)).into();
    let foot = container(open).width(Fill).align_right(Fill).padding([SPACE_2, SPACE_3]);
    body.push(hairline()).push(foot.boxed()).boxed()
}

fn flight_row<'a>(line: &'a RunLine, ctx: &'a Ctx) -> Element<'a, app::Message> {
    let name = container(text(line.name.as_str()).size(TEXT_BODY).style(theme::ink).wrapping(Wrapping::None)).width(Fill).clip(true);
    let mono = |s: String| text(s).font(theme::MONO).size(STATS_SIZE);
    let inner: Element<'a, app::Message> = match &line.outcome {
        None => row![name, mono("…".to_string()).style(theme::faint)].spacing(SPACE_2).align_y(Center).boxed(),
        Some(Ok(s)) => {
            let stats = format!(
                "{} · {} · {:.1} s",
                units::with_unit(s.apogee, Quantity::Altitude, ctx.units),
                units::with_unit(s.max_speed, Quantity::Velocity, ctx.units),
                s.flight_time
            );
            row![name, mono(stats).style(theme::muted)].spacing(SPACE_2).align_y(Center).boxed()
        }
        Some(Err(error)) => column![name, text(error.as_str()).size(STATS_SIZE).style(theme::colored(|t| t.err))].spacing(2).boxed(),
    };
    container(inner).width(Fill).padding([SPACE_2, SPACE_3]).boxed()
}
