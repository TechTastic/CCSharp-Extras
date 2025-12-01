using System;
using System.Collections.Generic;
using CCSharp.Attributes;
using CCSharp.RedIL.Enums;
using CCSharp.RedIL.Resolving.Attributes;
using CCSharp.RedIL.Resolving.CommonResolvers;

namespace CCSharp.RedIL.Resolving.Types;

class SystemConsoleResolverPack
{
    [RedILDataType(DataValueType.Unknown)]
    class ConsoleProxy
    {
        [LuaMethod("print")]
        public static void WriteLine() {}
        
        [LuaMethod("print")]
        public static void WriteLine<T>(T str) {}

        [LuaMethod("print")]
        public static void Write<T>(T str) {}
    }

    public static Dictionary<Type, Type> GetMapToProxy()
    {
        return new Dictionary<Type, Type>()
        {
            { typeof(Console), typeof(ConsoleProxy) }
        };
    }
}