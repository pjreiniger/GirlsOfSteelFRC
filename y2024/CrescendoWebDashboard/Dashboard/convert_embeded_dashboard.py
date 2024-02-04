from bs4 import BeautifulSoup
import os


def handle_tab(dashboard_tab):
    tab_name = dashboard_tab.get('tab-name')
    output_file = os.path.join(r'C:\Users\PJ\git\gos\GirlsOfSteelFRC\y2024\CrescendoWebdash3\src\tabs')

    if tab_name == "Drivers":
        output_file = os.path.join(output_file, "DriverTab.svelte")
    elif tab_name == "Pit":
        output_file = os.path.join(output_file, "PitTab.svelte")
    else:
        output_file = os.path.join(output_file, "subsystems", tab_name.replace(" ", "") + "SubsystemTab.svelte")

    if not os.path.exists(os.path.dirname(output_file)):
        os.mkdir(os.path.dirname(output_file))


    with open(output_file, 'w') as f:
        command_tags = dashboard_tab.find_all('frc-robot-command')
        if command_tags:
            f.write("<ul>\n")
            for ct in command_tags:
                ct['name'] = "UNLINKED"
                f.write("<li>\n")
                del ct["style"]
                ct['style'] = "width: 250px; height: 50px;"
                f.write(ct.prettify())
                f.write("\n")
                f.write("</li>\n")
            f.write("</ul>\n")

        for tag in dashboard_tab:
            if tag.name == 'frc-robot-command':
                continue
            if tag.name == "vaadin-number-field":
                continue
            if tag.name is None:
                print(f"WHY IS THIS NONE FOR {tab_name}")
                print("*" * 79)
                print(tag)
                print("*" * 79)
                continue

            del tag["style"]
            if tag.name == "frc-network-alerts":
                del tag["infos"]
                del tag["errors"]
                del tag["warnings"]

            if tag.name == "frc-sendable-chooser":
                del tag["options"]
                del tag["selected"]

            if tag.name == "frc-line-chart":
                if tab_name == "Arm Pivot":
                    tag["style"] = "transform-origin: 0 0; transform: translate(50%, -100%);"
                elif tab_name == "Shooter":
                    tag["style"] = "transform-origin: 0 0; transform: translate(50%, -50%);"

            try:
                f.write(tag.prettify())
                f.write("\n")
            except:
                print("FAILED")
                raise


def main():
    with open(r'/WebDash.html', 'r') as f:
        soup = BeautifulSoup(f.read(), 'html.parser' )

    for dashboard_tab in soup.find_all('dashboard-tab'):
        handle_tab(dashboard_tab)

if __name__ == "__main__":
    main()