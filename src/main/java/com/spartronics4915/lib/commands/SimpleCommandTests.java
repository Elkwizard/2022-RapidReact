package com.spartronics4915.lib.commands;

import static com.spartronics4915.lib.commands.SimpleCommands.Use.*;

public class SimpleCommandTests {
	public static void main(String[] args) {
		Do(() -> {
			System.out.println("first serial");
		}).Until(() -> false).Then(() -> {
			System.out.println("second serial");
		}).Then(
			Do(() -> {
				System.out.println("first parallel");
			}).And(() -> {
				System.out.println("second parallel");
			}).And(() -> {
				System.out.println("third parallel");
			})
		).Then(
			If(() -> false).Then(Do(() -> {
				System.out.println("How did we get here?");
			}))
		).run();

	}
}
