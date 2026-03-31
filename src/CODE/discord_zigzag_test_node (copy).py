import os
import discord
from discord.ext import commands
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DiscordZigzagBotNode(Node):
    def __init__(self):
        super().__init__('discord_zigzag_test_node')

        intents = discord.Intents.default()
        intents.message_content = True
        self.bot = commands.Bot(command_prefix="!", intents=intents)

        self.WORKSPACE_DIR = "/home/user/abd_ws_2"
        self.perimeter_dir = os.path.join(self.WORKSPACE_DIR, "src/tests/tests/perimeter")
        self.perimeter_name = None

        self.cmd_pub = self.create_publisher(String, '/perimeter_cmd', 10)
        self.name_pub = self.create_publisher(String, '/perimeter_name', 10)
        self.path_gen_trigger_pub = self.create_publisher(String, '/generate_path_trigger', 10)  
        self.nav_trigger_pub = self.create_publisher(String, '/stanley_start_cmd', 10)

        self.setup_bot_commands()

    def send_command(self, command_char):
        msg = String()
        msg.data = command_char
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Published: {command_char}")

    def send_name(self, name):
        msg = String()
        msg.data = name
        self.name_pub.publish(msg)
        self.get_logger().info(f"Published perimeter name: {name}")

    def send_navigation_trigger(self, action="start"):
        msg = String()
        msg.data = action
        self.nav_trigger_pub.publish(msg)
        self.get_logger().info(f"Published navigation trigger: {action}")

    def setup_bot_commands(self):
        @self.bot.event
        async def on_ready():
            print(f"🤖 Bot is ready. Logged in as {self.bot.user}.")

        @self.bot.command()
        async def start(ctx):
            await ctx.send("🚀 Welcome to the Autonomous Robot Controller!")
            await ctx.send("Here’s what you can do:")
            await ctx.send("""
1️⃣  !drawperimeter   → Start naming and recording perimeter
2️⃣  !send_y          → Start recording GPS points
3️⃣  !send_n          → Stop recording GPS points
4️⃣  !send_e          → Exit perimeter recording node
5️⃣  !generate_path   → Generate zigzag path from recorded perimeter
6️⃣  !confirm_start   → Start autonomous navigation
7️⃣  !stop_navigation → Stop the robot immediately
            """)
            await ctx.send("💡 Start with `!drawperimeter`.")

        @self.bot.command()
        async def help(ctx):
            await ctx.send("🤖 Here are all available commands:")
            await ctx.send("""
!start             → Show full list of actions to begin
!drawperimeter     → Start naming and drawing perimeter
!send_y            → Start recording GPS points
!send_n            → Stop recording
!send_e            → Exit recording mode
!generate_path     → Generate path from perimeter
!confirm_start     → Start autonomous navigation
!stop_navigation   → Stop the robot
            """)

        @self.bot.command()
        async def drawperimeter(ctx):
            await ctx.send("✏️ Please enter a name for your perimeter area (e.g., `field_1`).")

            def check(m):
                return m.author == ctx.author and m.channel == ctx.channel

            try:
                response = await self.bot.wait_for('message', check=check, timeout=60)
                self.perimeter_name = response.content.strip()
                self.send_name(self.perimeter_name)

                folder_path = os.path.join(self.perimeter_dir, self.perimeter_name)
                os.makedirs(folder_path, exist_ok=True)

                await ctx.send(f"📁 Perimeter name set: `{self.perimeter_name}`")
                await ctx.send("✅ Ready to record GPS points.")
                await ctx.send("👉 Use `!send_y` to start recording.")
                await ctx.send("🛑 Use `!send_n` to stop recording.")
                await ctx.send("🚪 Use `!send_e` when you are done.")

            except Exception:
                await ctx.send("⏳ Timeout! Please run `!drawperimeter` again.")

        @self.bot.command()
        async def send_y(ctx):
            await ctx.send("✏️ Starting GPS recording...")
            self.send_command('y')

        @self.bot.command()
        async def send_n(ctx):
            await ctx.send("🛑 Stopping GPS recording...")
            self.send_command('n')
            if self.perimeter_name:
                await ctx.send(f"📂 Data saved to `{os.path.join(self.perimeter_dir, self.perimeter_name)}`")

        @self.bot.command()
        async def send_e(ctx):
            await ctx.send("🚪 Exiting perimeter recording node...")
            self.send_command('e')
            await ctx.send("💾 You can now run `!generate_path` to process the GPS data.")

        @self.bot.command()
        async def generate_path(ctx):
            await ctx.send("🧙 Generating zigzag path from perimeter...")
            trigger_msg = String()
            trigger_msg.data = "start"
            self.path_gen_trigger_pub.publish(trigger_msg)
            await ctx.send("✅ Path generation complete. Run `!confirm_start` to begin navigation.")

        @self.bot.command()
        async def confirm_start(ctx):
            await ctx.send("📂 Enter the name of the perimeter you want to navigate:")

            def check(m):
                return m.author == ctx.author and m.channel == ctx.channel

            try:
                response = await self.bot.wait_for('message', check=check, timeout=60)
                chosen_perimeter = response.content.strip()
                path_dir = os.path.join(self.perimeter_dir, chosen_perimeter)

                if not os.path.exists(path_dir):
                    await ctx.send("❌ That perimeter does not exist! Please check the name.")
                    return

                self.perimeter_name = chosen_perimeter
                self.send_name(chosen_perimeter)
                self.send_command('start')
                self.send_navigation_trigger("start")

                await ctx.send(f"📆 Loading perimeter `{chosen_perimeter}` from `{path_dir}`")
                await ctx.send("✅ Robot is now navigating autonomously.")
                await ctx.send("📴 Use `!stop_navigation` to stop the robot.")

            except Exception:
                await ctx.send("⏳ Timeout! Please run `!confirm_start` again.")

        @self.bot.command()
        async def stop_navigation(ctx):
            self.send_navigation_trigger("stop")
            await ctx.send("🛑 Stop command sent to the robot.")
            await ctx.send("✅ If robot doesn't stop, confirm navigation node is subscribed to `/stanley_start_cmd`.")

    def run(self, token):
        self.bot.run(token)


def main(args=None):
    rclpy.init(args=args)
    node = DiscordZigzagBotNode()
    try:
      node.run("MTM3ODA2NjgyOTQwMjA1MDYzMA.GPC-c1.NdYLUzjSfH1jP4-dBZaSSX7SZ1SzYlCE2GrykY")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
